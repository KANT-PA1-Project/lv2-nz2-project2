# tutor 게시 검증

2026-11-05 KST 게시 준비본에서 확인했다.

- 주요 CPU 시험: **84 passed, 11 subtests passed**. capture lifecycle, observation owner, stationary GC, pick/place cycle, mission markers, scene coverage 대상.
- 모델/설정/자산 manifest: **67개 파일 크기·SHA-256 일치**. pickle/torch 모델을 역직렬화하지 않고 바이트 검증했다.
- 게시 파일의 일반 GitHub token/AWS access-key/private-key 패턴 검사에서 발견 없음. 원격 접속 비밀번호·대시보드 connection 설정·가상환경은 게시 대상에 포함하지 않았다.
- Git LFS로 모델과 큰 CAD 바이너리를 관리한다.
- 원본 실험/자산 소스의 일부 기존 trailing whitespace/EOF 공백은 해시·재현성을 위해 보존했다. 새 로직 실패로 간주하지 않는다.
- 테스트 환경의 pytest 플러그인 deprecation 경고 2개가 있었다. 전체 Isaac/ROS runtime 재현이나 전체 미션 합격 검증은 이번 게시 시험 범위가 아니다.

원격 PC는 모두 사용자 지시로 중단·재개 대기다. 어떤 과거 문서도 이 정책을 자동으로 되돌릴 수 없다.
