import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

# 1. 웹 페이지 기본 설정
st.set_page_config(page_title="미적분 PID 제어 시뮬레이터", layout="wide")
st.title("🚁 드론 고도 제어: 미적분 시뮬레이터")
st.markdown("왼쪽의 슬라이더를 움직여 **미분(기울기)**과 **적분(넓이)**의 힘을 조절하고, 드론의 고도 그래프가 어떻게 변하는지 확인해 보세요.")

# 2. 사이드바 (제어 변수 조절 UI)
st.sidebar.header("제어 상수 조절")
Kp = st.sidebar.slider("P: 비례 이득 (현재 오차)", 0.0, 2.0, 0.5, step=0.1)
Ki = st.sidebar.slider("I: 적분 이득 (누적 오차)", 0.0, 1.0, 0.0, step=0.01)
Kd = st.sidebar.slider("D: 미분 이득 (오차 변화율)", 0.0, 2.0, 0.0, step=0.1)
noise_level = st.sidebar.slider("외부 센서 노이즈", 0.0, 5.0, 0.0, step=0.5)

# 3. 물리 환경 및 초기값 세팅
target_altitude = 100.0  # 드론의 목표 고도
time_steps = 200         # 시뮬레이션 반복 횟수
dt = 0.1                 # 시간 간격 (1루프당 시간)

current_altitude = 0.0   # 드론의 현재 고도 (바닥에서 시작)
integral_error = 0.0     # 오차의 적분(누적) 변수
previous_error = target_altitude - current_altitude

times = []
actual_altitudes = []
target_altitudes = []

# 4. PID 제어 핵심 알고리즘 루프 (미적분 구현부)
for t in range(time_steps):
    # ① 오차 계산
    error = target_altitude - current_altitude
    
    # ② 적분 (구분구적법: 직사각형 넓이의 누적)
    integral_error += error * dt
    
    # ③ 미분 (순간 변화율: 오차의 기울기)
    derivative_error = (error - previous_error) / dt
    
    # ④ PID 최종 제어값 산출
    control_output = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error)
    
    # ⑤ 드론의 실제 움직임 업데이트 (출력값 + 노이즈 반영)
    current_altitude += control_output * dt
    current_altitude += np.random.normal(0, noise_level) # 불규칙한 바람(노이즈)
    
    # ⑥ 그래프 그리기 위해 데이터 저장
    times.append(t * dt)
    actual_altitudes.append(current_altitude)
    target_altitudes.append(target_altitude)
    
    # 다음 시간을 위해 오차 업데이트
    previous_error = error

# 5. 그래프 시각화
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(times, target_altitudes, 'r--', label='목표 고도 (Target)')
ax.plot(times, actual_altitudes, 'b-', linewidth=2, label='실제 드론 고도 (Actual)')
ax.set_xlabel('시간 (Time)')
ax.set_ylabel('고도 (Altitude)')
ax.set_title('시간에 따른 드론의 궤적 변화')
ax.legend()
ax.grid(True)

st.pyplot(fig)

st.success("수학적 결론: 적분(I)은 오차를 누적하여 끝까지 0으로 만들고, 미분(D)은 기울기를 계산하여 급격한 변화에 브레이크를 겁니다.")
