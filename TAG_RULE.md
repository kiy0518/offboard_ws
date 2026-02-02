# Git 태그 버전 관리 규칙

## 버전 형식: Semantic Versioning
- `vX.0.0` (Major) - 기존 호환 깨지는 큰 변경 (예: 아키텍처 변경, 멀티드론 지원)
- `v0.X.0` (Minor) - 새 기능 추가 (예: 웨이포인트 다중 경유, 새 제어 모드)
- `v0.0.X` (Patch) - 버그 수정, 파라미터 튜닝

## 릴리즈 노트 형식

```
vX.Y.Z - 제목

날짜: YYYY-MM-DD HH:MM:SS

[변경사항]
- 변경 내용 1
- 변경 내용 2

[이슈]
- 현재 알려진 문제점이나 제한사항
- 없으면 "없음" 표기

[향후 진행예정]
- 다음 버전에서 구현할 기능이나 개선사항
- 없으면 "없음" 표기
```

## 태그 생성 명령어

```bash
git tag -a vX.Y.Z -m "릴리즈 노트 내용"
git push origin vX.Y.Z
```

## 태그 삭제 (재생성 시)

```bash
git tag -d vX.Y.Z
git push origin --delete vX.Y.Z
```
