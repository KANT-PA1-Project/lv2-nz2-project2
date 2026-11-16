#!/usr/bin/env python3
"""팀원 CenterPose(centerpose_best.pt) → ONNX — YOLO 와 같은 순서로 (내보내기 → PyTorch 와 비교 → TensorRT) [2026-11-04]

왜 모델 코드를 여기서 다시 만드나:
    체크포인트에는 가중치(state_dict)만 있고 모델 정의 코드가 없음 → YOLO 처럼 `yolo export` 한 줄로 안 됨.
    가중치 이름·모양을 보면 CenterNet/CenterPose 의 DLA-34 (pose_dla_dcn) 와 똑같고,
    변형 합성곱(DCN) 자리에 오프셋 가중치가 없는 일반 3x3 합성곱(+bias)이 들어 있음 → TensorRT 플러그인 없이 변환 가능.
    그래서 원본 구조를 그대로 다시 만들고 가중치를 strict 로 불러 이름·모양이 하나도 안 틀리는지 확인한 뒤 내보냄.
구조: DLA-34(levels 1,1,1,2,2,1 / channels 16,32,64,128,256,512) → DLAUp → IDAUp(출력 stride 4, 64채널)
      → 헤드 7개 (3x3 64 + ReLU + 1x1): hm 1, wh 2, hps 16, reg 2, hm_hp 8, hp_offset 2, scale 3
출력: 원시 텐서 (sigmoid·디코딩·NMS 는 모델 밖, 원본 CenterPose 와 같음). 크기 = 입력 / 4
입력 계약 [2026-11-06, 팀 레포 tutor 브랜치 r12/scripts/export_r12_perception_onnx.py·quantize_r12_perception_onnx.py,
           같은 모델임을 SHA-256 으로 확인 (centerpose_best.pt = 3ed5fe57…)]:
    입력 [3, 3, 128, 128] = 한 물체를 crop 배율 1.5 / 1.35 / 1.65 로 자른 3장 (batch 3, 결과는 중앙값 'scale3_median')
    출력 4개만 사용: hm, hps, reg, scale  (wh, hm_hp, hp_offset 는 tutor 도 안 씀 → 빼면 계산·복사 줄어듦)
    정규화: RGB /255 [가정] (crop_image 원본 = 팀원 train_centerpose.py, 아직 못 봄). 정합 비교는 같은 텐서라 영향 없음
기본값 = 위 계약 (128x128, batch 3, 헤드 4개). 예전 방식은 --batch 1 --heads all --imgsz 256 256
실행 (개발 PC, yolo_env — torch, onnx, onnxruntime 필요):
    cd ~/git/p2_perception && source ~/yolo_env/bin/activate
    python optimize/centerpose_export.py                                # 128x128 b3 h4 (tutor 계약)
    python optimize/centerpose_export.py --imgsz 256 256 --batch 1 --heads all   # 예전 (헤드 7개)
결과: optimize/onnx_cp/centerpose_<H>x<W>[_b<B>][_h<N>].onnx   예: centerpose_128x128_b3_h4.onnx
      optimize/onnx_cp/ref_<같은 이름>/input.npy, reference_0~N-1.npy, names.json (출력 이름 순서)
      optimize/onnx_cp/centerpose_export_report.json
다음: python3 optimize/trt_bench.py --onnx optimize/onnx_cp/centerpose_128x128_b3_h4.onnx --tag pc
"""
import argparse
import json
import os
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PT = "drive-download-20261104T022039Z-1-001/centerpose_best.pt"
HEADS = {"hm": 1, "wh": 2, "hps": 16, "reg": 2, "hm_hp": 8, "hp_offset": 2, "scale": 3}   # 체크포인트 순서


def build_model(torch, nn):
    BN_MOMENTUM = 0.1

    class BasicBlock(nn.Module):
        def __init__(self, inplanes, planes, stride=1, dilation=1):
            super().__init__()
            self.conv1 = nn.Conv2d(inplanes, planes, 3, stride, padding=dilation, bias=False, dilation=dilation)
            self.bn1 = nn.BatchNorm2d(planes, momentum=BN_MOMENTUM)
            self.relu = nn.ReLU(inplace=True)
            self.conv2 = nn.Conv2d(planes, planes, 3, 1, padding=dilation, bias=False, dilation=dilation)
            self.bn2 = nn.BatchNorm2d(planes, momentum=BN_MOMENTUM)

        def forward(self, x, residual=None):
            if residual is None:
                residual = x
            out = self.relu(self.bn1(self.conv1(x)))
            out = self.bn2(self.conv2(out))
            return self.relu(out + residual)

    class Root(nn.Module):
        def __init__(self, cin, cout, k, residual):
            super().__init__()
            self.conv = nn.Conv2d(cin, cout, 1, stride=1, bias=False, padding=(k - 1) // 2)
            self.bn = nn.BatchNorm2d(cout, momentum=BN_MOMENTUM)
            self.relu = nn.ReLU(inplace=True)
            self.residual = residual

        def forward(self, *x):
            children = x
            x = self.bn(self.conv(torch.cat(x, 1)))
            if self.residual:
                x = x + children[0]
            return self.relu(x)

    class Tree(nn.Module):
        def __init__(self, levels, block, cin, cout, stride=1, level_root=False, root_dim=0, root_kernel_size=1,
                     dilation=1, root_residual=False):
            super().__init__()
            if root_dim == 0:
                root_dim = 2 * cout
            if level_root:
                root_dim += cin
            if levels == 1:
                self.tree1 = block(cin, cout, stride, dilation=dilation)
                self.tree2 = block(cout, cout, 1, dilation=dilation)
                self.root = Root(root_dim, cout, root_kernel_size, root_residual)
            else:
                self.tree1 = Tree(levels - 1, block, cin, cout, stride, root_dim=0, root_kernel_size=root_kernel_size,
                                  dilation=dilation, root_residual=root_residual)
                self.tree2 = Tree(levels - 1, block, cout, cout, root_dim=root_dim + cout,
                                  root_kernel_size=root_kernel_size, dilation=dilation, root_residual=root_residual)
            self.level_root = level_root
            self.levels = levels
            self.downsample = nn.MaxPool2d(stride, stride=stride) if stride > 1 else None
            self.project = nn.Sequential(nn.Conv2d(cin, cout, 1, stride=1, bias=False),
                                         nn.BatchNorm2d(cout, momentum=BN_MOMENTUM)) if cin != cout else None

        def forward(self, x, residual=None, children=None):
            children = [] if children is None else children
            bottom = self.downsample(x) if self.downsample is not None else x
            residual = self.project(bottom) if self.project is not None else bottom
            if self.level_root:
                children.append(bottom)
            x1 = self.tree1(x, residual)
            if self.levels == 1:
                x2 = self.tree2(x1)
                return self.root(x2, x1, *children)
            children.append(x1)
            return self.tree2(x1, children=children)

    class DLA(nn.Module):
        def __init__(self, levels, channels):
            super().__init__()
            self.channels = channels
            self.base_layer = nn.Sequential(nn.Conv2d(3, channels[0], 7, stride=1, padding=3, bias=False),
                                            nn.BatchNorm2d(channels[0], momentum=BN_MOMENTUM), nn.ReLU(inplace=True))
            self.level0 = self._conv_level(channels[0], channels[0], levels[0])
            self.level1 = self._conv_level(channels[0], channels[1], levels[1], stride=2)
            self.level2 = Tree(levels[2], BasicBlock, channels[1], channels[2], 2, level_root=False)
            self.level3 = Tree(levels[3], BasicBlock, channels[2], channels[3], 2, level_root=True)
            self.level4 = Tree(levels[4], BasicBlock, channels[3], channels[4], 2, level_root=True)
            self.level5 = Tree(levels[5], BasicBlock, channels[4], channels[5], 2, level_root=True)

        @staticmethod
        def _conv_level(inplanes, planes, convs, stride=1):
            mods = []
            for i in range(convs):
                mods += [nn.Conv2d(inplanes, planes, 3, stride=stride if i == 0 else 1, padding=1, bias=False),
                         nn.BatchNorm2d(planes, momentum=BN_MOMENTUM), nn.ReLU(inplace=True)]
                inplanes = planes
            return nn.Sequential(*mods)

        def forward(self, x):
            y = []
            x = self.base_layer(x)
            for i in range(6):
                x = getattr(self, f"level{i}")(x)
                y.append(x)
            return y

    class DeformConvAsConv(nn.Module):
        """원본 DeformConv(BN+ReLU, DCN) 자리. 체크포인트에 DCN 오프셋 가중치가 없고 conv.weight·conv.bias 만 있음"""
        def __init__(self, chi, cho):
            super().__init__()
            self.actf = nn.Sequential(nn.BatchNorm2d(cho, momentum=BN_MOMENTUM), nn.ReLU(inplace=True))
            self.conv = nn.Conv2d(chi, cho, 3, stride=1, padding=1, bias=True)

        def forward(self, x):
            return self.actf(self.conv(x))

    class IDAUp(nn.Module):
        def __init__(self, o, channels, up_f):
            super().__init__()
            for i in range(1, len(channels)):
                c, f = channels[i], int(up_f[i])
                setattr(self, f"proj_{i}", DeformConvAsConv(c, o))
                setattr(self, f"up_{i}", nn.ConvTranspose2d(o, o, f * 2, stride=f, padding=f // 2, output_padding=0,
                                                            groups=o, bias=False))
                setattr(self, f"node_{i}", DeformConvAsConv(o, o))

        def forward(self, layers, startp, endp):
            for i in range(startp + 1, endp):
                layers[i] = getattr(self, f"up_{i - startp}")(getattr(self, f"proj_{i - startp}")(layers[i]))
                layers[i] = getattr(self, f"node_{i - startp}")(layers[i] + layers[i - 1])

    class DLAUp(nn.Module):
        def __init__(self, startp, channels, scales):
            super().__init__()
            self.startp = startp
            in_channels = list(channels)
            channels = list(channels)
            scales = np.array(scales, dtype=int)
            for i in range(len(channels) - 1):
                j = -i - 2
                setattr(self, f"ida_{i}", IDAUp(channels[j], in_channels[j:], scales[j:] // scales[j]))
                scales[j + 1:] = scales[j]
                in_channels[j + 1:] = [channels[j] for _ in channels[j + 1:]]

        def forward(self, layers):
            out = [layers[-1]]
            for i in range(len(layers) - self.startp - 1):
                getattr(self, f"ida_{i}")(layers, len(layers) - i - 2, len(layers))
                out.insert(0, layers[-1])
            return out

    class CenterPoseDLA34(nn.Module):
        def __init__(self, heads, down_ratio=4, last_level=5, head_conv=64):
            super().__init__()
            self.first_level = int(np.log2(down_ratio))
            self.last_level = last_level
            self.base = DLA([1, 1, 1, 2, 2, 1], [16, 32, 64, 128, 256, 512])
            ch = self.base.channels
            self.dla_up = DLAUp(self.first_level, ch[self.first_level:],
                                [2 ** i for i in range(len(ch[self.first_level:]))])
            self.ida_up = IDAUp(ch[self.first_level], ch[self.first_level:self.last_level],
                                [2 ** i for i in range(self.last_level - self.first_level)])
            self.heads = list(heads)
            for h, c in heads.items():
                setattr(self, h, nn.Sequential(nn.Conv2d(ch[self.first_level], head_conv, 3, padding=1, bias=True),
                                               nn.ReLU(inplace=True), nn.Conv2d(head_conv, c, 1, bias=True)))

        def forward(self, x):
            x = self.dla_up(self.base(x))
            y = [x[i].clone() for i in range(self.last_level - self.first_level)]
            self.ida_up(y, 0, len(y))
            return tuple(getattr(self, h)(y[-1]) for h in self.heads)   # ONNX 출력 = 헤드 순서

    return CenterPoseDLA34(HEADS)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pt", default=DEFAULT_PT)
    ap.add_argument("--imgsz", type=int, nargs=2, default=[128, 128], metavar=("H", "W"))
    ap.add_argument("--imgsz2", type=int, nargs=2, action="append", default=[], metavar=("H", "W"),
                    help="추가 크기 (여러 번)")
    ap.add_argument("--batch", type=int, default=3, help="crop 장수 (tutor: 배율 3개 = 3)")
    ap.add_argument("--heads", nargs="+", default=["hm", "hps", "reg", "scale"],
                    help="내보낼 출력 (all = 7개 전부)")
    ap.add_argument("--opset", type=int, default=17)
    ap.add_argument("--out", default="optimize/onnx_cp")
    a = ap.parse_args()

    import onnxruntime as ort
    import torch
    import torch.nn as nn

    pt = a.pt if os.path.isabs(a.pt) else os.path.join(ROOT, a.pt)
    out = a.out if os.path.isabs(a.out) else os.path.join(ROOT, a.out)
    os.makedirs(out, exist_ok=True)
    ck = torch.load(pt, map_location="cpu", weights_only=False)
    sd = ck["model"] if isinstance(ck, dict) and "model" in ck else ck
    info = {k: v for k, v in ck.items() if k != "model"} if isinstance(ck, dict) else {}
    print(f"체크포인트: {pt}\n  정보: {info}")

    model = build_model(torch, nn)
    res = model.load_state_dict(sd, strict=False)
    missing = [k for k in res.missing_keys]
    unexpected = [k for k in res.unexpected_keys]
    print(f"  가중치 불러오기: 빠진 키 {len(missing)}, 남는 키 {len(unexpected)}")
    if missing or unexpected:
        print("   빠진:", missing[:10], "\n   남는:", unexpected[:10])
        raise SystemExit("구조가 체크포인트와 다름 → 위 키 목록을 붙여 주세요 (구조를 맞춤)")
    model.eval()
    heads = list(HEADS) if a.heads == ["all"] else a.heads
    bad = [h for h in heads if h not in HEADS]
    if bad:
        raise SystemExit(f"없는 헤드 {bad} (가능: {list(HEADS)} 또는 all)")
    idx = [list(HEADS).index(h) for h in heads]

    class Pick(nn.Module):                     # 필요한 헤드만 출력 (안 쓰는 헤드 계산은 그래프에서 빠짐)
        def __init__(self, m):
            super().__init__()
            self.m = m

        def forward(self, x):
            y = self.m(x)
            return tuple(y[i] for i in idx)

    net = Pick(model).eval() if len(heads) < len(HEADS) else model
    n_param = sum(p.numel() for p in model.parameters())
    print(f"  구조 일치 (strict), 파라미터 {n_param:,}")

    report = {"pt": pt, "checkpoint_info": {k: str(v) for k, v in info.items()}, "params": n_param,
              "heads": heads, "batch": a.batch, "opset": a.opset, "sizes": {}}
    for h, w in [tuple(a.imgsz)] + [tuple(s) for s in a.imgsz2]:
        if h % 32 or w % 32:
            print(f"[{h}x{w}] 32 의 배수가 아님 → 건너뜀")
            continue
        print(f"\n===== {h}x{w} =====", flush=True)
        tag = f"{h}x{w}" + (f"_b{a.batch}" if a.batch != 1 else "") + (f"_h{len(heads)}" if len(heads) < len(HEADS) else "")
        dst = os.path.join(out, f"centerpose_{tag}.onnx")
        # 0~1 난수 (RGB /255 입력 범위와 같게 → FP16 비교가 실제 값 범위에서 이뤄지도록)
        x = torch.from_numpy(np.random.default_rng(0).random((a.batch, 3, h, w)).astype(np.float32))
        t0 = time.time()
        kw = dict(opset_version=a.opset, input_names=["input"], output_names=heads, do_constant_folding=True)
        try:
            torch.onnx.export(net, x, dst, dynamo=False, **kw)      # 예전 방식(TorchScript) 내보내기 고정
        except TypeError:                                             # torch 2.4 이하는 dynamo 인자 없음
            torch.onnx.export(net, x, dst, **kw)
        mb = os.path.getsize(dst) / 1e6
        print(f"  ONNX {dst} ({mb:.1f} MB, {time.time() - t0:.0f} s)")
        try:
            import onnx
            onnx.checker.check_model(onnx.load(dst))
        except ImportError:
            pass
        sess = ort.InferenceSession(dst, providers=["CPUExecutionProvider"])
        with torch.no_grad():
            yt = [t.numpy() for t in net(x)]
        yo = sess.run(None, {"input": x.numpy()})
        rows = {}
        for name, a_, b_ in zip(heads, yt, yo):
            rows[name] = dict(shape=list(b_.shape), max_abs=float(np.max(np.abs(a_ - b_))),
                              ref_absmax=float(np.max(np.abs(a_))))
            print(f"  {name:10s} {str(list(b_.shape)):18s} 최대 오차 {rows[name]['max_abs']:.2e} "
                  f"(값 크기 최대 {rows[name]['ref_absmax']:.2f})")
        # 실습 compare_outputs.py 용 파일 (ONNX 출력 순서 = reference_i)
        rdir = os.path.join(out, f"ref_{tag}")
        os.makedirs(rdir, exist_ok=True)
        json.dump(heads, open(os.path.join(rdir, "names.json"), "w"))
        np.save(os.path.join(rdir, "input.npy"), x.numpy())
        for i, t in enumerate(yt):
            np.save(os.path.join(rdir, f"reference_{i}.npy"), t)
        report["sizes"][tag] = dict(onnx=dst, mb=round(mb, 2), outputs=rows, ref_dir=rdir)

    rp = os.path.join(out, "centerpose_export_report.json")
    json.dump(report, open(rp, "w"), indent=1, ensure_ascii=False)
    print("\n================ 요약 ================")
    for k, v in report["sizes"].items():
        worst = max(r["max_abs"] for r in v["outputs"].values())
        print(f"{k:>16s} {v['mb']:6.1f} MB  출력 {len(heads)}개 최대 오차 {worst:.2e}  → {'OK' if worst < 1e-3 else '확인 필요'}")
    print(f"저장: {rp}")
    print(f"다음: python3 optimize/trt_bench.py --onnx {dst} --tag pc  (YOLO 와 같음)")


if __name__ == "__main__":
    main()
