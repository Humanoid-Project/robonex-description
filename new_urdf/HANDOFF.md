# RoboNex Ver.2 URDF 인수인계

이 폴더는 Fusion 360 Ver.2 모델에서 URDF를 새로 만드는 작업의 결과물이다. 작업 범위는 **URDF 작성까지**이며,
`robonex-description`과 `robonex-common`은 한 줄도 수정하지 않았다. 통합, 빌드, 검증은 이 문서를 받는 세션이 한다.
상세 수치와 방법은 `REPORT.md`에 있다.

## 1. 내용물

| 경로 | 내용 |
|---|---|
| `urdf/robonex.urdf` | 링크 34개, 조인트 33개(revolute 29, fixed 4), 26.241 kg. 메시 경로는 `../meshes/*.stl` |
| `loop_closures.yaml` | 기존과 같은 구조. 볼 루프 4개, 핀 루프 2개, `ball_upgrades` 4개, ±15°, `actuated_joints` 12개(Ver.1과 동일) |
| `meshes/` | STL 56개(약 60 MB). mm 단위, URDF 축(X 앞, Y 왼쪽, Z 위), **각 링크 프레임 기준이라 visual origin이 모두 0** |
| `REPORT.md` | 추출 방법, 검증 결과, Ver.1 비교 |
| `_extract/` | Fusion 추출 원본(JSON, raw STL)과 생성 스크립트. `python make_meshes.py && python make_urdf.py`로 재생성(numpy만 필요) |

`2_RoboNex_test.f3z`와 `RoboNex_부품별_질량표.docx`는 사용자가 넣어 둔 파일이다. 이번 작업에는 쓰지 않았다.
URDF 질량은 모두 Fusion 물성값이다(RS05만 보정). 질량표의 실측값과 대조하는 작업은 하지 않았다.

출처는 다음과 같다.
- 하체와 머리: Fusion `PolyGon/Ver.2/2_RoboNex_test v1`
- 팔: 위 문서의 복사본 `Ver.2/2_RoboNex_urdf`. 팔꿈치만 as-built 조인트로 앞으로 90° 회전해서 저장했고, 나머지 55개 부품은 원본과 동일함을 확인했다.

## 2. 그대로 지킨 계약

- **링크·조인트 이름:** Ver.1 이름 25개와 24개를 그대로 쓴다. 발목의 `crank_link_a`/`coupler_joint_a` 좌우 규칙도 Ver.1과 같다.
- **영점과 프레임:** 모든 조인트의 `rpy`가 0이고, 영점에서 모든 링크 프레임이 `base_link`와 평행하다. 축은 ±X/±Y/±Z이고 다리는 편 자세다.
- **구동 12개의 축 부호:** Ver.1과 전부 같다. "URDF 축 = 모터 출력 방향의 반대" 규칙이 Ver.1 12개에서 모두 성립했고, Ver.2 모터에 적용해도 같은 부호가 나온다. 즉 모터를 반대로 다시 단 곳이 없다.
- **질량 묶음:** 모터는 하우징이 볼트로 고정된 링크에 포함한다. Fusion 관성 추출 파이프라인은 Ver.1 URDF 값을 1e-8까지 재현했다.
- **폐루프 표현:** Ver.1과 같다. 크랭크와 수동 관절은 URDF 트리에 넣고, 무릎 핀과 발목 볼은 `loop_closures.yaml`에 적는다. 발목 로드엔드는 URDF에서 `fixed`이고 빌더가 볼조인트로 바꾼다. 영점 루프 잔차는 1e-6 m 이하다.

## 3. Ver.1과 달라진 점 (통합 시 주의)

1. **메시 방식:** Ver.1은 부품 중심 기준 STL에 visual `origin`의 rpy를 썼다. 이번에는 링크 프레임 기준 STL에 원점 0을 쓴다. 파일 이름이 같은 37개도 내용과 좌표계가 다르므로 `meshes/`를 **통째로 교체**해야 한다. 섞어 쓰면 안 된다.
2. **새로 추가된 링크·조인트 (Ver.1에 없음):**
   - 머리: `neck_pitch_joint`(RS05)와 `head_link`(머리와 RealSense를 메시 하나로 합침). `base_link`에 `rs05_neck`, `neck_mount` 메시가 추가됨.
   - 팔(Unitree G1 이름과 체인, 접두사만 `l_`/`r_`): `*_shoulder_pitch/roll/yaw_joint`, `*_elbow_joint`, 링크 `*_shoulder_pitch/roll/yaw_link`, `*_elbow_link`. 모터 메시 이름은 `rs02_*_shoulder_pitch` 방식.
   - **팔 영점은 팔꿈치를 앞으로 90° 굽힌 자세**다(G1과 같음).
   - 이 관절 9개는 `robonex-common`에 없고 `actuated_joints`에도 넣지 않았다. CAN ID, 게인, 한계, 모터 방향이 모두 미정이다.
3. **관절 한계:** 사용자 요청으로 revolute 29개 모두 −3.141593~+3.141593이다. effort와 velocity는 모터 스펙이다: RS02 17 / 42.9, RS03 60 / 20.9, RS05 5.5 / 50.3(공식 480 rpm), 수동 관절 0 / 0. `robonex-common`의 한계와 정책 action 정규화는 아직 Ver.1 값이다.
4. **무릎 기구가 새 설계다:**
   - 4절 링크: 크랭크 70.0, 커플러 134.0, 로커 92.4, 고정 링크 175.0 mm (Ver.1: 100 / 134 / 103 / 85).
   - 무릎 전달비 d(knee)/d(crank): 영점에서 1.000(Ver.1 0.7037), 크랭크 −30°에서 0.734, −60°에서 0.431.
   - **크랭크를 펴는 방향으로 +20.0°에서 링키지가 한계점에 도달한다.** Ver.1 한계 +54°는 도달할 수 없다.
   - 따라서 skill reference(`robonex.md`)의 무릎 수치, `serial_model.py`의 무릎 effort 85.1과 한계, `ros2_control.yaml`의 무릎 게인이 모두 Ver.1 전달비 기준이라 맞지 않는다.
5. **다리 길이 변화:**
   - hip pitch에서 발목 pitch까지 663.5 mm(Ver.1 750.5).
   - base 원점에서 발바닥까지 약 950 mm(Ver.1 1078.9).
   - 발목 모터 간격 101 mm(Ver.1 111). 로드 147 / 258 mm와 크랭크 반경 50 mm는 Ver.1과 같다.
6. **질량:** 총 26.241 kg(Ver.1 20.514). 하체 21.375, 머리 쪽 0.609, 팔 4.257 kg이다. RS05는 Fusion 재질 밀도 오류(95.6 g)를 공식 191 g으로 보정했다.

## 4. 현재 조립 상태를 그대로 영점으로 쓴 결과 (결정 필요)

영점은 "Fusion 조립 상태 그대로"다. 조립에 다음 비대칭과 기울기가 있어서 URDF에도 그대로 들어갔다.

- 왼발이 pitch 약 1.0°(앞끝이 들림), roll 0.3° 기울어져 있다. 오른발은 거의 수평이다.
- 발목 크랭크 볼의 각도(수평 기준)가 좌우 다르다. 왼쪽 upper 12.78°, lower 1.85°이고, 오른쪽 upper 11.90°, lower 0.35°다. 로드 길이 차이(111 mm)와 모터 간격(101 mm)이 달라서 발이 평평하려면 크랭크가 비대칭이어야 한다.
- 비대칭: hip yaw 축이 y +43.073 / −42.927 mm로 0.146 mm 어긋나 있고, 오른팔은 좌우 대칭 위치보다 약 0.5 mm 안쪽에 있다.
- **이 영점이 RobStride Set Zero 기준이 된다.** 하드웨어 영점 설정과 관절 한계 sweep 전에 영점 자세(특히 발목 크랭크 각도와 왼발 수평)를 확정해야 한다.

## 5. `robonex-description` 빌더에 넣을 때 예상되는 문제

| 대상 | 문제 | 조치 |
|---|---|---|
| `mujoco/build_mjcf.py` | `HOME_HEIGHT` 1.0710과 `HOME_PASSIVE_JOINT_POS`가 Ver.1 값이라, free-base 빌드에서 home 키프레임 잔차 검사가 실패한다 | `--fixed-base` 빌드, `home_pose.py` 실행, 상수 갱신, free-base 빌드 순서로 진행 |
| `build_mjcf.py` | 팔과 목 관절이 `actuated_joints`에 없어서 actuator 없는 passive hinge가 되고, 중력에 늘어진다 | 게인이 정해질 때까지 고정하거나 액추에이터 정의를 추가 |
| `build_mjcf.py` | 구동 관절 range가 ±π라서 폐루프가 무릎 한계점(+20°)이나 로드엔드 ±15°를 넘을 수 있다(safety skill의 2026-08-13 발산 사고와 같은 조건) | 한계를 측정하기 전에는 모델 보기 용도로만 쓰고, 학습이나 무작위 동작 테스트는 하지 않는다 |
| `robonex-urdf` skill `home_pose.py` | 수동 관절을 "`DEFAULT_JOINT_POS`에 없는 hinge"로 판별하므로 팔과 목 hinge도 수동 관절로 잡혀 출력 dict에 섞인다 | 출력에서 팔·목 키를 제외하고 반영 |
| `home_pose.py`, walking 계약 | `FOOT_SOLE_CORNERS`와 `FOOT_ORIGIN_REST_HEIGHT`가 Ver.1 발 기준이다. 발 부품이 바뀌었다(833 g, 메시 최저점이 발 프레임 z −66.66(L) / −65.89(R) mm, x −75~166, y ±65 mm) | 새 발 메시에서 발바닥 모서리를 다시 산출. 왼발 기울기 때문에 좌우 값이 다르다 |
| `isaac/build_isaac_urdf.py` | `MOTOR_JOINTS`가 다리 12개로 고정되어 있어 팔과 목 관절이 `fixed`로 나간다 | 의도라면 그대로 두고, 아니면 목록 추가 |
| `gazebo/`, `ros2/serial_model.py` | `SERIAL_JOINTS`, `SERIAL_GAINS`, `SERIAL_FRICTION`의 무릎·발목 직렬 상수가 Ver.1 전달비와 한계 기준이다. 팔과 목은 `fixed`로 나간다 | 새 무릎 전달비로 다시 계산 |
| `scripts/robonex_data.py` | `COLLISION_BOX`, `SPAWN_HEIGHT`(1.0789), `MUJOCO_SPAWN_HEIGHT`(1.085)가 Ver.1 값이다 | 영점 높이 약 0.9501(왼발 최저 정점 기준), 스폰 약 0.956. 박스 값은 `_extract/derived.py` 출력 사용(하체 링크만) |
| `length-coupled-constants.md` 전체 | base height, stance width, 넘어짐 판정 높이, 접촉력 한계(질량 증가), Froude 주석, gait period 등 | skill 목록대로 전부 갱신 |

## 6. 확인되지 않은 것

- 팔 RS02 8개와 목 RS05의 **축 부호**: 다리 규칙을 적용해서 정했을 뿐 하드웨어로 확인하지 않았다(목: +방향 = 고개를 듦).
- MuJoCo는 URDF를 그대로 로딩하는 것까지만 확인했다(34 bodies, 56 meshes). `build_mjcf.py`, `home_pose.py`, Isaac 변환·`apply_physical_loops.py`, 발목 가동범위와 로드엔드 ±15° sweep은 실행하지 않았다(Ubuntu와 `robonex-common` 필요).
- 영점에서의 메시 겹침: 루프로 연결된 쌍(커플러와 정강이, 로드와 발)과 부모-자식 쌍뿐이며 `build_mjcf.py`의 이웃 제외 규칙으로 처리된다. MuJoCo는 메시를 볼록 껍질로 충돌 처리하므로, 몸통처럼 큰 메시는 어깨 roll이나 yaw를 움직일 때 실제보다 일찍 충돌한다.
- 메시가 무겁다: `base_link` 11.9만, `head_link`(RealSense 포함) 10.4만, RS03 3.6만, RS05 7.2만 삼각형, 총 60 MB. 로딩이 느리면 decimation을 검토한다.

## 7. Fusion 쪽 상태

- `2_RoboNex_test v1`: 수정 없음(읽기 전용 스크립트만 사용).
- `2_RoboNex_urdf`: 팔꿈치 as-built 조인트 2개와 위치 캡처를 저장한 복사본. 팔을 다시 추출할 때 이 문서를 쓴다.
- `00_RoboNex v9`(Ver.1): 수정 없음.
- Fusion MCP 경험: 무거운 스크립트(면 단위 거리 측정)는 Fusion을 멈추게 한다. 부품 하나씩 가벼운 스크립트로 실행해야 한다. STL 내보내기는 컴포넌트 로컬 좌표로 저장되므로 occurrence 변환행렬을 적용해야 한다. 스크립트 문자열에 한글이 들어가면 깨진다.
