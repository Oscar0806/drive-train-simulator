import numpy as np
from scipy.integrate import odeint
 
# === DC Motor Parameters (typical industrial 5kW motor) ===
DEFAULT_PARAMS = {
    "Ra": 0.5,        # armature resistance (Ohm)
    "La": 0.001,      # armature inductance (H)
    "Kt": 0.4,        # torque constant (Nm/A)
    "Kb": 0.4,        # back-EMF constant (V*s/rad)
    "J_motor": 0.05,  # motor inertia (kg*m^2)
    "B_motor": 0.001, # motor friction (Nm*s/rad)
    "gear_ratio": 10, # gearbox reduction
    "gear_eff": 0.95, # gearbox efficiency
    "J_load": 2.0,    # load inertia reflected (kg*m^2)
    "B_load": 0.05,   # load friction (Nm*s/rad)
}
 
def reflected_inertia(p):
    """Total inertia reflected to motor side."""
    n = p["gear_ratio"]
    return p["J_motor"] + p["J_load"] / (n**2 * p["gear_eff"])
 
def reflected_friction(p):
    """Total friction reflected to motor side."""
    n = p["gear_ratio"]
    return p["B_motor"] + p["B_load"] / (n**2 * p["gear_eff"])
 
def motor_dynamics(state, t, V_in, T_load, p):
    """DC motor + gearbox + load coupled dynamics."""
    i_a, omega_motor = state
    J_eq = reflected_inertia(p)
    B_eq = reflected_friction(p)
    
    # Electrical equation: V = i*R + L*di/dt + Kb*omega
    di_dt = (V_in - p["Ra"]*i_a - p["Kb"]*omega_motor) / p["La"]
    
    # Mechanical equation: J*domega/dt = Kt*i - B*omega - T_load_reflected
    T_load_reflected = T_load / (p["gear_ratio"] * p["gear_eff"])
    domega_dt = (p["Kt"]*i_a - B_eq*omega_motor - T_load_reflected) / J_eq
    
    return [di_dt, domega_dt]
 
def pi_speed_controller(omega_setpoint, omega_actual, integral_state, Kp, Ki, dt, V_max=400):
    """PI controller for speed regulation with anti-windup."""
    error = omega_setpoint - omega_actual
    integral_state += error * dt
    V_command = Kp * error + Ki * integral_state
    
    # Saturation + anti-windup (back-calculation)
    if V_command > V_max:
        V_command = V_max
        integral_state = (V_max - Kp*error) / Ki if Ki != 0 else integral_state
    elif V_command < -V_max:
        V_command = -V_max
        integral_state = (-V_max - Kp*error) / Ki if Ki != 0 else integral_state
    
    return V_command, integral_state
 
def simulate_drive_train(t_span, omega_setpoint_rpm, T_load_profile, p, Kp=2.0, Ki=15.0):
    """Closed-loop simulation of drive train with PI speed control."""
    n_steps = len(t_span)
    dt = t_span[1] - t_span[0]
    
    omega_setpoint = omega_setpoint_rpm * 2 * np.pi / 60  # convert to rad/s
    
    states = np.zeros((n_steps, 2))  # [i_a, omega_motor]
    voltages = np.zeros(n_steps)
    torques_motor = np.zeros(n_steps)
    integral_state = 0.0
    
    state = [0.0, 0.0]  # initial conditions
    
    for k in range(n_steps - 1):
        # Get setpoint at current time (can be time-varying)
        sp_now = omega_setpoint[k] if hasattr(omega_setpoint, '__len__') else omega_setpoint
        
        # PI controller
        V_in, integral_state = pi_speed_controller(
            sp_now, state[1], integral_state, Kp, Ki, dt
        )
        
        T_load_now = T_load_profile[k]
        
        # Integrate one step
        sol = odeint(motor_dynamics, state, [t_span[k], t_span[k+1]], 
                     args=(V_in, T_load_now, p))
        state = sol[-1]
        
        states[k+1] = state
        voltages[k+1] = V_in
        torques_motor[k+1] = p["Kt"] * state[0]
    
    # Output side (after gearbox)
    omega_load = states[:, 1] / p["gear_ratio"]  # rad/s
    omega_load_rpm = omega_load * 60 / (2 * np.pi)
    omega_motor_rpm = states[:, 1] * 60 / (2 * np.pi)
    torque_load = torques_motor * p["gear_ratio"] * p["gear_eff"]
    
    return {
        "t": t_span,
        "i_a": states[:, 0],
        "omega_motor_rpm": omega_motor_rpm,
        "omega_load_rpm": omega_load_rpm,
        "voltage": voltages,
        "torque_motor": torques_motor,
        "torque_load": torque_load,
    }
 
if __name__ == "__main__":
    p = DEFAULT_PARAMS.copy()
    t = np.linspace(0, 5, 1000)
    setpoint_rpm = np.ones(1000) * 1000  # 1000 rpm setpoint
    T_load = np.ones(1000) * 5  # 5 Nm load
    
    result = simulate_drive_train(t, setpoint_rpm, T_load, p)
    print(f"Final motor speed: {result['omega_motor_rpm'][-1]:.1f} rpm")
    print(f"Final load speed:  {result['omega_load_rpm'][-1]:.1f} rpm")
    print(f"Final current:     {result['i_a'][-1]:.2f} A")
    print(f"Steady-state voltage: {result['voltage'][-1]:.1f} V")