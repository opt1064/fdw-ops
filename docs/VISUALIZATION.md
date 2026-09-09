# Isaac Sim 그래픽 표출 가이드 (AGX Thor)

FDW-OPS PoC-1 시뮬레이션을 시각적으로 확인하는 4가지 방법을 정리한 문서입니다.

## 4가지 방법 비교

| 방법 | 성능 | 원격 가능 | 설정 난이도 | 추천 상황 |
|------|------|-----------|-------------|-----------|
| 로컬 GUI | ⭐⭐⭐⭐⭐ | ❌ | 쉬움 | AGX Thor 본체 앞에서 작업 |
| WebRTC Livestream | ⭐⭐⭐⭐ | ✅ | 보통 | 원격 데모, 다중 시청자 |
| SSH X11 | ⭐ | ✅ | 쉬움 | 간단한 디버깅만 |
| VNC | ⭐⭐ | ✅ | 보통 | 전체 데스크톱 공유 필요 |

## 방법 1 — 로컬 GUI (모니터 직결)

AGX Thor에 모니터가 연결되어 있을 때 가장 간단하고 빠른 방법입니다.

```bash
conda activate isaac_sim
cd ~/isaac_workspace/projects/fdw-sim

ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1.py --mode isaac --gui
```

- Isaac Sim 창이 모니터에 직접 표시
- 30~60 FPS의 매끄러운 시뮬레이션
- Viewport, Property, Stage 패널 등 모든 GUI 기능 사용 가능

## 방법 2 — WebRTC Livestream (원격 권장)

브라우저 또는 전용 클라이언트로 어디서든 접속 가능. Isaac Sim 5.x의 공식 원격 표출 방식입니다.

### AGX Thor에서 실행

```bash
conda activate isaac_sim
cd ~/isaac_workspace/projects/fdw-sim

ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1.py \
    --mode isaac \
    --livestream 2
```

`--livestream 2`를 사용하면 자동으로 headless로 실행되며 WebRTC 서버가 활성화됩니다.

### 클라이언트에서 접속

원격 PC의 브라우저에서:
```
http://<AGX_THOR_IP>:8211/streaming/webrtc-client
```

또는 NVIDIA Streaming Client 설치 후 `<AGX_THOR_IP>` 입력.

### 방화벽 포트

```bash
sudo ufw allow 8211/tcp   # WebRTC HTTP
sudo ufw allow 49100:49200/udp  # WebRTC media
```

대역폭은 약 10~30 Mbps 권장.

## 방법 3 — Native Livestream (Streaming Client)

```bash
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1.py \
    --mode isaac \
    --livestream 1
```

NVIDIA Omniverse Streaming Client (Windows/Linux)를 설치한 PC에서 접속.

## 방법 4 — SSH X11 Forwarding

3D 렌더링이 매우 느려 비실용적이지만 디버깅 용도로는 가능합니다.

```bash
# 원격 PC에서
ssh -X isweon@<AGX_THOR_IP>

# AGX Thor에서
conda activate isaac_sim
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1.py --mode isaac --gui
```

## 방법 5 — VNC (전체 데스크톱)

```bash
# AGX Thor에서 VNC 서버 설치 (최초 1회)
sudo apt install -y tigervnc-standalone-server

# VNC 서버 시작
vncserver :1 -geometry 1920x1080

# 원격 PC에서 VNC 클라이언트로 접속
# Address: <AGX_THOR_IP>:5901
```

VNC 세션 안에서 터미널을 열고 `--gui` 모드로 실행.

## 빠른 명령어 치트시트

| 상황 | 명령 |
|------|------|
| 본체 앞 GUI | `... run_poc1.py --mode isaac --gui` |
| 원격 WebRTC | `... run_poc1.py --mode isaac --livestream 2` |
| 헤드리스 (시각화 없음) | `... run_poc1.py --mode isaac --headless` |
| 빠른 로직 검증 | `... run_poc1.py --mode discrete` |

모든 명령은 다음 prefix가 필요:
```
ACCEPT_EULA=Y PRIVACY_CONSENT=Y python scripts/run_poc1.py
```

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| 검은 화면만 표시 | GPU 드라이버 미인식 | `nvidia-smi`로 GPU 확인, X 세션 재시작 |
| WebRTC 연결 안됨 | 방화벽 차단 | 포트 8211, 49100~49200 개방 |
| 첫 실행 1~3분 멈춤 | 셰이더 컴파일 | 정상, 기다림. `~/.cache/ov` 크기 증가 모니터링 |
| `vulkan layer not found` | Vulkan 미설치 | `sudo apt install vulkan-tools` |
| `Failed to acquire interface` | 메모리 부족 | `headless` + `livestream 2` 조합 사용, 다른 프로세스 종료 |

## 권장 워크플로우

1. **개발 초기**: Discrete 모드로 빠르게 로직 검증 (`--mode discrete`)
2. **시각 확인**: 본체 앞에서 GUI 모드로 첫 동작 확인 (`--mode isaac --gui`)
3. **원격 협업/데모**: WebRTC로 시연 (`--mode isaac --livestream 2`)
4. **연속 실험**: 헤드리스로 백그라운드 실행 (`--mode isaac --headless`)
