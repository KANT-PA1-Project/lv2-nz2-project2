#!/usr/bin/env python3
"""카메라 시야 사전 점검 (Isaac 불필요). 실패하면 종료 코드 1
실행: bash ~/manipulator-isaac61-lyrical-practice/scripts/nz2_vispick_run.sh geom"""
import json, os, sys
sys.path.insert(0, os.path.expanduser("~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks"))
import nz2_vispick_geom as G
r = G.report(float(os.environ.get("NZ2_TOPCAM_PITCH_DEG", G.TOP_CAM_PITCH_DEG_DEFAULT)))
print(json.dumps(r, ensure_ascii=False, indent=1))
print("결과:", "통과" if r["ok"] else "실패")
sys.exit(0 if r["ok"] else 1)
