# Ubuntu 24.04 Docker·Isaac ROS 5.0 준비

여기부터는 학습용 가상환경에서 나와 **새 호스트 터미널**에서 실행합니다. Isaac ROS는 Docker 모드 하나로 진행합니다. 별도로 호스트에 ROS 2를 설치할 필요가 없습니다.

```bash
# 현재 터미널에 .venv-yolo가 활성화되어 있을 때만 실행
deactivate
```

### 7-1. Docker Engine

이미 Docker가 설치되어 있다면 `docker version`으로 확인합니다. Docker Desktop·배포판의 `docker.io`와 Docker CE가 섞인 PC는 [Docker 공식 안내](https://docs.docker.com/engine/install/ubuntu/)의 충돌 패키지 절차를 먼저 확인합니다. 아래는 새 Ubuntu PC의 Docker CE 설치 경로입니다.

```bash
sudo apt update
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
sudo tee /etc/apt/sources.list.d/docker.sources > /dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: noble
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker "$USER"
```

로그아웃·로그인 후 새 터미널에서 확인합니다. Docker 그룹은 관리자 수준의 권한을 갖습니다. 수업용 PC의 사용 계정에만 부여합니다.

```bash
docker run --rm hello-world
```

### 7-2. 컨테이너의 GPU 사용 설정

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --gpus all ubuntu:24.04 nvidia-smi
```

마지막 명령에서 호스트와 같은 GPU가 보이면 통과입니다. Docker를 재시작하면 실행 중인 다른 컨테이너에 영향을 줄 수 있으므로 작업 중인 컨테이너가 없는 때 진행합니다. [NVIDIA Container Toolkit 설치](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## 8. Isaac ROS 5.0 환경 만들기 — 호스트에서 시작

### 8-1. 작업 폴더와 CLI 저장소

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
export ISAAC_ROS_WS="$HOME/workspaces/isaac_ros-dev"
sudo apt install -y curl gnupg software-properties-common
sudo add-apt-repository -y universe
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-isaac-ros.gpg
echo 'deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-5.0 noble main' \
  | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list
sudo apt update
sudo apt install -y isaac-ros-cli
sudo isaac-ros init docker
```

기존 Isaac ROS를 사용하던 PC에서는 저장소·CLI 설정을 확인하고 기존 프로젝트와 충돌하지 않는지 먼저 확인합니다. 이 문서는 release-5.0으로 고정하며 release-3.2의 `run_dev.sh` 절차를 섞지 않습니다. 컨테이너 모드에서는 호스트에 Lyrical Buildfarm·CUDA Toolkit을 추가로 설치하는 bare-metal 절차를 따라 하지 않습니다. [공식 설치](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html)
