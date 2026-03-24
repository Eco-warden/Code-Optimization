# Code-Optimization

## **성능 최적화**

- `filtered_points.reserve()` 추가 → 벡터 재할당 방지
- `for (auto point : ...)` → `for (const auto& pt : ...)` 전환 → 불필요한 복사 제거
- `clusters.push_back(current_cluster)` → `std::move()` 적용 → 복사 비용 제거
- `ComputeClusterCenter()` 함수 분리 → 중심 계산과 폭 계산을 한 곳에서 처리, 중복 코드 제거
- `std::accumulate` + 람다로 중심 좌표 합산 → 명시적 반복문 대체

## **코드 품질 개선**

- 모든 매직 넘버(`50.0f`, `400.0f`, `30.0f` 등)를 `constexpr` 상수로 추출 → 수치 변경 시 한 곳만 수정
- TCP 분기의 중복 `if` 제거 → `else`로 통합
- 구조체 필드 기본값 명시 초기화 (`center_x = 0.0f` 등)
