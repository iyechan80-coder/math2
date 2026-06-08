import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

# =====================================================================
# 1. 웹 페이지 기본 설정 및 헤더
# =====================================================================
st.set_page_config(page_title="미적분 PID 제어 시뮬레이터", layout="wide")
st.title("🚁 드론 고도 제어: 미적분 시뮬레이터")
st.markdown("왼쪽의 슬라이더를 움직여 **미분(기울기)**과 **적분(넓이)**의 힘을 조절하고, 현실의 중력과 노이즈 속에서 드론이 어떻게 비행하는지 확인해 보세요.")

# =====================================================================
# 2. 사이드바 (제어 변수 조절 UI)
# =====================================================================
st.sidebar.header("제어 상수 조절")
Kp = st.sidebar.slider("P: 비례 이득 (현재 오차)", 0.0, 5.0, 1.0, step=0.1)
Ki = st.sidebar.slider("I: 적분 이득 (누적 오차)", 0.0, 2.0, 0.0, step=0.01)
Kd = st.sidebar.slider("D: 미분 이득 (오차 변화율)", 0.0, 5.0, 0.0, step=0.1)
noise_level = st.sidebar.slider("외부 센서 노이즈 (바람)", 0.0, 5.0, 0.0, step=0.5)

# =====================================================================
# 3. 물리 환경 및 초기값 세팅
# =====================================================================
target_altitude = 100.0  # 목표 고도
time_steps = 300         # 시뮬레이션 반복 횟수 (시간)
dt = 0.1                 # 시간 간격

current_altitude = 0.0   # 드론의 현재 고도 (바닥에서 시작)
integral_error = 0.0     # 오차의 누적합 (적분) 변수
previous_error = target_altitude - current_altitude

# 그래프를 그리기 위한 데이터 저장 리스트
times = []
actual_altitudes = []
target_altitudes = []

# =====================================================================
# 4. PID 제어 핵심 알고리즘 루프 (미적분 구현부)
# =====================================================================
for t in range(time_steps):
    # ① 현재 오차 계산
    error = target_altitude - current_altitude
    
    # ② 적분 (구분구적법: 직사각형 넓이의 누적)
    integral_error += error * dt
    
    # [심화: 안티 와인드업] 물리적 한계 반영 (적분 폭주 방지)
    if integral_error > 500: integral_error = 500
    elif integral_error < -500: integral_error = -500
    
    # ③ 미분 (순간 변화율: 오차의 기울기)
    derivative_error = (error - previous_error) / dt
    
    # ④ PID 최종 제어 출력값 산출
    control_output = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error)
    
    # ⑤ 현실 물리 엔진 적용 (모터 출력 + 중력 + 바람)
    current_altitude += control_output * dt
    current_altitude -= 2.0 * dt  # 🚨 중력 작용 (P 제어만으로 100m 도달을 막는 잔류 편차 원인)
    current_altitude += np.random.normal(0, noise_level) # 외부 바람(노이즈) 작용
    
    # 드론이 땅(0m)을 뚫고 내려가지 않도록 물리적 하한선 설정
    if current_altitude < 0: 
        current_altitude = 0

    # ⑥ 시각화를 위한 데이터 저장
    times.append(t * dt)
    actual_altitudes.append(current_altitude)
    target_altitudes.append(target_altitude)
    
    # 다음 루프를 위해 오차 업데이트
    previous_error = error

# =====================================================================
# 5. 상단 결과 대시보드 (수치 요약)
# =====================================================================
col1, col2, col3 = st.columns(3)
col1.metric("최종 고도 (Final Altitude)", f"{current_altitude:.1f} m")
col2.metric("최종 오차 (Final Error)", f"{abs(error):.1f} m")
col3.metric("최고 고도 (Max Overshoot)", f"{max(actual_altitudes):.1f} m")
st.divider()

# =====================================================================
# 6. 그래프 시각화 
# (클라우드 서버의 한글 폰트 깨짐 현상 방지를 위해 영어 라벨 사용)
# =====================================================================
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(times, target_altitudes, 'r--', label='Target Altitude (100m)')
ax.plot(times, actual_altitudes, 'b-', linewidth=2, label='Actual Drone Altitude')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Altitude (m)')
ax.set_title('Drone Altitude PID Control Simulation')
ax.legend(loc='lower right')
ax.grid(True)

st.pyplot(fig)

# 7. 탐구 결론 요약
st.success("💡 **시뮬레이션 조작 팁:** P(비례)만 올리면 중력을 이기지 못해 100m 아래에서 멈춥니다(잔류 편차 발생). 이때 I(적분)를 올리면 편차가 면적으로 누적되어 기어코 100m로 끌어올립니다. 마지막으로 외부 노이즈를 주고 D(미분)를 올려 미분이 어떻게 기계를 폭주시키는지 확인해 보세요.")
