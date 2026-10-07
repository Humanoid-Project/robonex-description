# Ver.2-2 Edu 작업 보고서 (2026-10-07 05:00 예약 작업)

작업 지시서: `TASK_0500.md`. 사용자 질문 없이 지시서의 기본값으로 진행했다.
원본 `Ver.2-2/2_RoboNex_clean`은 수정하지 않았다 (마지막 확인 시점에도 v1).

## 결과물
| 항목 | 위치 | 상태 |
|---|---|---|
| Fusion 작업 파일 | `Humanoid_Project/Ver.2-2/Ver2-2_Edu_RoboNex` **v14** (id `urn:adsk.wipprod:dm.lineage:UdKZJ4rpSkWgv9pW65h08w`) | 저장됨, 수정 없음 |
| STL 39개 | `new2_urdf/meshes/*.stl` (바이너리, mm, 링크 좌표계) | 완료 |
| URDF | `new2_urdf/ver2-2_Edu.urdf` | 완료, MuJoCo 로드 확인 |
| 루프 정의 | `new2_urdf/loop_closures.yaml` (`new_urdf`의 것과 바이트 단위 동일) | 복사 |
| 작업 스크립트·중간 데이터 | `new2_urdf/_work/` (`progress.json`, `baseline.json`, `targets.json`, `joints_spec.json`, `urdf_inertials.json`, `render_zero_pose.png` 등) | 보관 |

Fusion 저장 이력:
- v1: saveAs 복사
- v2: 이름 변경
- v3: 질량
- v4~v7: 좌표계 정렬 (11개 → 23개 → 38개 → 39개)
- v8: 접지, 조인트 27개
- v9: 조인트 44개 (검증 완료)
- v10: base_link 2,936.90 g (CAN 모듈·USB 허브 추가, 2026-10-07 사용자 결정)
- v11: `CAD 부품별 질량.docx` 기준으로 하체 18개 부품 질량 변경 (총 25.0161 kg)
- v12: CAN 모듈은 base_link(2,856.90 g), USB 허브 2개는 battery_link(3,881.94 g)로 나눔 (사용자 결정)
- v13: ankle_crank_link_a/b 4개 76.8 → 97.70 g (링크암 67.70 + 볼조인트 30, 실측 기준, 사용자 결정). 커플러–발 쪽 볼조인트는 foot 940 g에 포함 (사용자 확인)
- v14: l/r_hip_roll_link 1,229.55 → 923.9 g (사용자 지정). 총 24.4884 kg. CAD 질량표 docx도 갱신 (이전 판 `_work/docx_backup/CAD 부품별 질량_v13.docx`, 이전 URDF `_work/ver2-2_Edu_v13.urdf`)

## 한 일
1. **복사·이름 변경**: `baselink_battery` → `battery_link`, `e-stop button` → `estop_link`로 바꿨다. 나머지 36개는 이미 URDF 이름이었다.
2. **질량 설정** (매핑 md의 확정 항목만, 부품별 전용 재질의 밀도로 맞춤):
   - base_link 2,828.90 g → **2,936.90 g** (CAN 모듈 28 g + USB 허브 2개 80 g 포함, 사용자 결정), battery_link 3,801.94 g, estop_link 603.00 g
   - RS02 6개 405 → **410 g**, RS03 6개 880 → **928 g** (재질 "RobStride RS02 410g" / "RobStride RS03 928g")
   - hip_pitch_link 301.65 → 305.65 g, knee_crank_link 152.61 → 154.61 g (좌우 각각)
   - hip_yaw_link 178, ankle_crank_link_a/b 97.70은 이미 같은 값이었다.
   - (v11) 이후 `CAD 부품별 질량.docx`의 부품별 값으로 다시 설정 → 아래 "CAD 질량표 반영" 참고.
   - **Fusion 총질량 24.4884 kg** = URDF 총질량 = MuJoCo 총질량 (v14).
3. **좌표계 정렬** (39개, 부품 1개씩): 컴포넌트 좌표계를 URDF 링크 좌표계에 맞췄다. 매번 질량·질량중심·관성이 그대로이고 다른 부품이 움직이지 않았음을 확인했다.
   - 새 링크의 원점은 질량 설정 후의 각 부품 질량중심이다. 축은 base_link와 평행하다. 값은 URDF base 기준이다.
     - `battery_link` (10.870, 0.164, -28.358) mm
     - `estop_link` (-102.180, 0.160, 103.793) mm
4. **base_link 접지**.
5. **조인트 44개** (하나씩 만들고, 만들 때마다 확인):
   - 구성은 Ver2_Edu_RoboNex와 같은 42개에 `battery_joint`, `estop_joint`(rigid, 자식 → base_link)를 더한 것이다.
   - 42개는 참조 모델에서 다음 항목을 읽어 그대로 재현했다: 조인트 종류, 부품 순서, 축 방향, 기하 원점.
   - 각 조인트를 만든 직후 확인한 항목:
     - 축 벡터가 참조와 일치 (내적 1.0000)
     - 기하 원점 오차 0.0001 mm 이하
     - URDF 부호 규칙에 따른 부호가 모두 +1 (반대 부호 0개)
     - 부품 이동 없음
   - 최종 검증 결과:
     - 부품 39개, 조인트 44개 (누락·순서 오류·억제 0)
     - 스케치 0, 타임라인 오류 0
     - 접지는 base_link
6. **STL 39개**: medium 정밀도로 내보냈다 (base_link만 low, 아래 참고).
   - 공통 36개는 `humanoid_project/meshes`의 기존 파일과 비교했다. 다시 만든 rs02_r_hip_yaw를 포함해 **삼각형 수와 경계상자가 모두 일치**한다 (차이 0.0000 mm).
   - base_link는 형상이 바뀌었고, battery/estop은 새 부품이라 비교 대상이 없다.
7. **URDF** `ver2-2_Edu.urdf`:
   - 링크 27개: Edu 25개 + battery_link, estop_link
   - 조인트 26개: robonex.urdf의 Edu 조인트 24개를 그대로 복사 (origin, axis, limit, dynamics 포함) + fixed 2개
   - 링크 관성은 Fusion 질량 속성으로 새로 계산했다. 계산 방법: 소속 부품을 평행축 정리로 합친 뒤 URDF 축으로 바꾸고, 단위를 m와 kg·m²로 맞췄다.
   - 질량이 바뀌지 않은 링크(knee_coupler, ankle_link, ankle_coupler, foot)는 기존 robonex.urdf와 비교했다: 질량중심 차이 0.002 mm 이내, 관성 차이 약 4e-9 이하. 계산 방식을 검증한 결과다.
   - 메쉬 경로는 `meshes/<이름>.stl`, scale 0.001이다. base_link 링크는 base_link와 rs02 hip yaw 2개로 이루어져 있다.
   - MuJoCo 3.11 확인 결과:
     - 바디 27, 회전 조인트 20, 메쉬 39
     - 전체 질량중심이 Fusion과 0.0001 mm 이내
     - battery/estop 바디의 질량중심이 각 원점과 일치
     - 발바닥 최저점 z: 왼쪽 -950.11 mm, 오른쪽 -949.34 mm
     - 렌더: `_work/render_zero_pose.png`

## 링크 질량 (URDF)
| 링크 | 질량 (g) | 질량중심 (링크 좌표계, mm) |
|---|---|---|
| `base_link` | 3,676.90 | 13.15 -5.92 -5.74 |
| `battery_link` | 3,881.94 | 0 0 0 |
| `estop_link` | 603.00 | 0 0 0 |
| `l/r_hip_yaw_link` | 1,106.00 | (L) -0.03 1.47 -60.17 |
| `l/r_hip_pitch_link` | 1,233.65 | (L) -5.31 60.40 -31.67 |
| `l/r_hip_roll_link` | 1,851.90 | |
| `l/r_knee_crank_link` | 176.61 | |
| `l/r_knee_coupler_link` | 89.60 | |
| `l/r_knee_link` | 2,306.94 | |
| `l/r_ankle_crank_link_a/b` | 97.70 | |
| `l/r_ankle_coupler_link_a` | 66.80 | |
| `l/r_ankle_coupler_link_b` | 90.40 | |
| `l/r_ankle_link` | 106.00 | |
| `l/r_foot` | 940.00 | |

질량은 모터를 포함한 링크 단위 값이다. 링크별 전체 값은 `_work/urdf_inertials.json`에 있다.

## 지시서와 다르게 하거나 판단한 것 (확인 부탁)
1. **rs02_r_hip_yaw를 새 컴포넌트로 다시 만들었다.**
   - 원인: 이 부품은 원본 타임라인의 맨 처음(0번)에 있다. 그래서 배치(transform)를 바꾼 뒤 스냅샷을 찍으면 원래 배치로 돌아갔고, 순서도 옮길 수 없었다.
   - 첫 시도에서는 이동이 잘못 적용됐다. 이때는 저장하지 않고 닫은 뒤 다시 열었다.
   - 해결 방법:
     - 링크 좌표계 위치에 새 컴포넌트를 만들었다.
     - 원래 바디를 월드 위치 그대로 기준 피처(base feature)로 복사하고, 재질 "RobStride RS02 410g"·외관·바디 이름도 그대로 옮겼다.
     - 원래 컴포넌트를 지우고, 새 컴포넌트 이름을 `rs02_r_hip_yaw`로 바꿨다.
   - 확인 결과:
     - 질량·질량중심·관성이 원본과 같다 (오차 1e-14 수준).
     - STL 삼각형 수와 경계상자가 기존 메쉬와 같다.
     - 이 과정에서 타임라인 항목 5개가 함께 지워졌지만 다른 부품의 배치는 바뀌지 않았다.
2. **base_link.stl만 low 정밀도로 내보냈다.**
   - 이유: medium으로 내보내면 삼각형이 205,938개로, MuJoCo STL 한도(200,000개)를 넘어 로드되지 않는다.
   - low는 88,260개이고 경계상자는 같다.
   - medium 원본은 `_work/base_link_medium.stl`에 보관했다. 다른 시뮬레이터용으로 medium이 필요하면 바꿔 넣으면 된다.
3. `r_ankle_ball_c/d`의 Fusion 조인트 기하가 커플러 쪽 구면이 아니라 발(r_foot) 쪽 구면(r 6.35)에 잡혔다.
   - 같은 반지름, 같은 중심이고, 참조 원점과 0.0000 mm 차이라 동작은 같다.
4. 조인트 limit는 robonex.urdf 그대로다 (구동 관절 -π~+π). Ver.2 limit sweep 결과가 나오면 바꿔야 한다.

## CAD 질량표 반영 (2026-10-07, v11)
- 기준: `CAD 부품별 질량.docx` (부품 이름별 질량), 참고: `개별부품 질량.docx` (실측 개별 부품).
- 바꾼 부품 18개 (좌우 각각, 단위 g):

  | 부품 | 이전 | 새 값 |
  |---|---|---|
  | hip_roll_link | 827.90 | 1,229.55 |
  | knee_crank_link | 154.61 | 176.61 |
  | knee_coupler_link | 27.60 | 89.60 |
  | knee_link | 677.00 | 1,486.941 |
  | ankle_crank_link_a, _b | 97.70 | 76.80 → v13에서 97.70으로 되돌림 (실측 링크암 + 볼조인트) |
  | ankle_coupler_link_a | 61.80 | 66.80 |
  | ankle_coupler_link_b | 95.40 | 90.40 |
  | foot | 833.00 | 940.00 |

- 액추에이터, hip_yaw_link 178, hip_pitch_link 305.65, ankle_link 106, estop 603은 이미 표와 같았다.
- 결과: 39개 중 37개가 표와 정확히 같다. 다른 2개는 다음과 같다.
  - base_link: 표 2,828.90, Fusion 2,856.90. CAN 모듈 28 g을 더했다 (v12, 사용자 결정).
  - battery_link: 표 3,801.9, Fusion 3,881.94. USB 허브 2개 80 g을 더했다 (v12). 0.04 g은 표의 반올림 차이다.
- 질량은 부품마다 전용 재질의 밀도로 맞췄다. 그래서 각 부품의 질량중심과 배치는 그대로이고, 관성은 질량 비율로만 바뀌었다.
- 표의 소계와 합계 행은 행 값과 맞지 않는다. 예를 들어 무릎·정강이 행을 더하면 3,506.30이지만 표에는 3,153.082로 적혀 있다. 그래서 소계 대신 각 행의 값을 썼다. 사용자 확인(2026-10-07): 행을 더한 값이 맞다. 행 합계 24,908.10 g에 CAN·USB 108 g을 더하면 Fusion 총질량 25,016.15 g이 된다(반올림 차이 0.04 g 포함).
- 이전 URDF는 `_work/ver2-2_Edu_v10.urdf`에 있다.

## 갱신 기록
- 2026-10-07: base_link에 CAN·USB 108 g 추가 → Fusion v10, URDF 다시 생성. 기준값을 정렬이 끝난 모델에서 다시 읽어서 ankle_crank_link 관성이 5e-8 kg·m² 정도(최대 0.2%) 함께 바뀌었다. 이전 URDF는 `_work/ver2-2_Edu_before_can_usb.urdf`에 있다.
- 2026-10-07: 질량표 docx 2개 수정 (원본은 `_work/docx_backup/`, 수정 스크립트 `_work/fix_docx.py`).
  - `CAD 부품별 질량.docx`: 값을 Fusion v13과 같게 맞췄다 (ankle_crank 97.70, base_link 2,856.90, battery_link 3,881.94, Ver.2-2 부품 이름). 소계·합계·비율을 다시 계산했다 (합계 25,099.742 g). 제목 앞 "30"을 지우고 출처·비고를 고쳤다. 몸체 프로파일·Orin이 "없다"던 문장은 base_link에 포함된다는 내용으로 바꿨다.
  - `개별부품 질량.docx`: 발목 링크암 포함 구성에 "볼조인트 1"을 적었다. 볼조인트 행은 하체 표에서 6번(개당 참고값)으로 옮겨 중복을 없앴다. 하체 소계 4,515.101 g, 합계 12,447.831 g, 비율을 다시 계산했다.

## 남은 일
- 조인트 limit 반영.
- `.f3z` 파일 내보내기는 사용자가 한다 (기존 방식).
- robonex-description, robonex-skill 저장소는 읽기만 했고 수정하지 않았다.
