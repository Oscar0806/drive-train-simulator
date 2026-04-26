import streamlit as st
import plotly.graph_objects as go
import numpy as np
from drivetrain_engine import DEFAULT_PARAMS, simulate_drive_train
 
st.set_page_config(page_title="Drive Train Simulator", page_icon="\u2699\uFE0F", layout="wide")
st.title("\u2699\uFE0F Industrial Drive Train Simulator — Motor + Gearbox + Load")
st.markdown("**Industrial Drive Technology Specialization**")
st.divider()
 
st.sidebar.header("\U0001F50C Motor Parameters")
Ra = st.sidebar.slider("Armature resistance Ra (\u03A9)", 0.1, 5.0, 0.5, 0.1)
La = st.sidebar.slider("Armature inductance La (mH)", 0.5, 10.0, 1.0, 0.5) / 1000
Kt = st.sidebar.slider("Torque constant Kt (Nm/A)", 0.1, 1.0, 0.4, 0.05)
 
st.sidebar.header("\u2699\uFE0F Mechanical")
gear_ratio = st.sidebar.select_slider("Gearbox ratio", [5, 10, 20, 50, 100], value=10)
gear_eff = st.sidebar.slider("Gear efficiency", 0.7, 0.99, 0.95, 0.01)
J_load = st.sidebar.slider("Load inertia (kg\u00B7m\u00B2)", 0.1, 10.0, 2.0, 0.1)
B_load = st.sidebar.slider("Load friction (Nm\u00B7s/rad)", 0.0, 0.5, 0.05, 0.01)
 
st.sidebar.header("\U0001F3AF Speed Setpoint")
target_rpm = st.sidebar.slider("Target speed (rpm)", 100, 3000, 1000, 50)
 
st.sidebar.header("\u2696\uFE0F PI Controller")
Kp = st.sidebar.slider("Proportional gain Kp", 0.1, 10.0, 2.0, 0.1)
Ki = st.sidebar.slider("Integral gain Ki", 0.1, 50.0, 15.0, 0.5)
 
st.sidebar.header("\u26A1 Load Disturbance")
T_load_steady = st.sidebar.slider("Steady load torque (Nm)", 0, 20, 5)
disturb_on = st.sidebar.checkbox("Add load step disturbance at t=2s", value=False)
disturb_mag = st.sidebar.slider("Disturbance magnitude (Nm)", -20, 30, 10) if disturb_on else 0
 
# Build parameters
p = DEFAULT_PARAMS.copy()
p.update({"Ra": Ra, "La": La, "Kt": Kt, "Kb": Kt,
          "gear_ratio": gear_ratio, "gear_eff": gear_eff,
          "J_load": J_load, "B_load": B_load})
 
t = np.linspace(0, 5, 1000)
setpoint_rpm = np.ones(1000) * target_rpm
T_load_profile = np.ones(1000) * T_load_steady
if disturb_on:
    T_load_profile[400:] += disturb_mag  # step at t=2s (index 400)
 
result = simulate_drive_train(t, setpoint_rpm, T_load_profile, p, Kp=Kp, Ki=Ki)
 
c1, c2, c3, c4 = st.columns(4)
final_motor = result["omega_motor_rpm"][-1]
final_load = result["omega_load_rpm"][-1]
err = abs(target_rpm - final_motor)
peak_curr = abs(result["i_a"]).max()
with c1: st.metric("Motor Speed", f"{final_motor:.0f} rpm", f"{err:.1f} error")
with c2: st.metric("Load Speed", f"{final_load:.0f} rpm")
with c3: st.metric("Peak Current", f"{peak_curr:.2f} A")
with c4: st.metric("Peak Voltage", f"{abs(result['voltage']).max():.1f} V")
st.divider()
 
col1, col2 = st.columns(2)
with col1:
    st.subheader("\U0001F4C8 Speed Response")
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=t, y=result["omega_motor_rpm"], name="Motor speed", line=dict(color="#2563EB",width=2)))
    fig.add_trace(go.Scatter(x=t, y=result["omega_load_rpm"], name="Load speed (after gearbox)", line=dict(color="#DC2626",width=2)))
    fig.add_hline(y=target_rpm, line_dash="dash", line_color="green", annotation_text="Setpoint")
    fig.update_layout(xaxis_title="Time (s)", yaxis_title="Speed (rpm)", height=350, template="plotly_white")
    st.plotly_chart(fig, use_container_width=True)
 
with col2:
    st.subheader("\u26A1 Current & Voltage")
    fig2 = go.Figure()
    fig2.add_trace(go.Scatter(x=t, y=result["i_a"], name="Armature current (A)", line=dict(color="#7C3AED",width=2)))
    fig2.add_trace(go.Scatter(x=t, y=result["voltage"]/10, name="Voltage / 10 (V)", line=dict(color="#F59E0B",width=2)))
    fig2.update_layout(xaxis_title="Time (s)", yaxis_title="Current (A) / Voltage (V/10)", height=350, template="plotly_white")
    st.plotly_chart(fig2, use_container_width=True)
 
st.subheader("\u2699\uFE0F Torque Profile")
col3, col4 = st.columns(2)
with col3:
    fig3 = go.Figure()
    fig3.add_trace(go.Scatter(x=t, y=result["torque_motor"], name="Motor torque", line=dict(color="#10B981",width=2)))
    fig3.add_trace(go.Scatter(x=t, y=result["torque_load"], name="Load torque (after gearbox)", line=dict(color="#EF4444",width=2)))
    fig3.update_layout(xaxis_title="Time (s)", yaxis_title="Torque (Nm)", height=300, template="plotly_white")
    st.plotly_chart(fig3, use_container_width=True)
with col4:
    st.markdown("**Drive Train Analysis**")
    st.markdown(f"- Motor inertia: {p['J_motor']} kg\u00B7m\u00B2")
    st.markdown(f"- Reflected total inertia: {p['J_motor'] + p['J_load']/(p['gear_ratio']**2 * p['gear_eff']):.4f} kg\u00B7m\u00B2")
    st.markdown(f"- Gear ratio: {gear_ratio}:1")
    st.markdown(f"- Gear efficiency: {gear_eff*100:.0f}%")
    st.markdown(f"- Power output: ~{abs(result['torque_load'][-1] * result['omega_load_rpm'][-1] * 2 * np.pi / 60):.0f} W")
    st.markdown(f"- Time constant (electrical): \u03C4_e = La/Ra = {1000*La/Ra:.2f} ms")
st.divider()
st.caption("Industrial Drive Train Simulator | Oscar Vincent Dbritto")