import numpy as np

from robot_tracking_project.kinematics import forward_kinematics, jacobian


def pseudoinverse_control(J, error, gain=1.0):
    return np.linalg.pinv(J) @ (gain * error)


def dls_control(J, error, damping=0.1, gain=1.0):
    identity = np.eye(J.shape[0])
    return J.T @ np.linalg.inv(J @ J.T + (damping ** 2) * identity) @ (gain * error)

def run_controller( mode='pinv', link_lengths=None, q_init=None, target=None, gain=1.0, 	 damping=0.1, dt=0.05, tolerance=0.01,max_steps=200,):
	if link_lengths is None:
        link_lengths = np.array([0.5, 0.7, 0.5], dtype=float)
    else:
        link_lengths = np.asarray(link_lengths, dtype=float)

    if q_init is None:
        q = np.array([0.2, 0.3, -0.2], dtype=float)
    else:
        q = np.asarray(q_init, dtype=float)

    if target is None:
        target = np.array([0.6, 0.2, 0.8], dtype=float)
    else:
        target = np.asarray(target, dtype=float)

    if q.shape != (3,):
        raise ValueError("q_init must be a length-3 vector.")
    if link_lengths.shape != (3,):
        raise ValueError("link_lengths must be a length-3 vector.")
    if target.shape != (3,):
        raise ValueError("target must be a length-3 vector.")
    if dt <= 0:
        raise ValueError("dt must be positive.")
    if tolerance <= 0:
        raise ValueError("tolerance must be positive.")
    if max_steps <= 0:
        raise ValueError("max_steps must be positive.")
    if damping <= 0:
        raise ValueError("damping must be positive for DLS.")
        
    positions = []
    errors = []
    q_history = [q.copy()]
    qdot_history = []

    for step in range(max_steps):
        x = forward_kinematics(q, link_lengths)
        error = target - x
        error_norm = np.linalg.norm(error)
        J = jacobian(q, link_lengths)

        positions.append(x.copy())
        errors.append(error_norm)

        if error_norm < tolerance:
            print(f"Target reached in {step} steps.")
            break

        if mode == 'pinv':
            q_dot = pseudoinverse_control(J, error, gain=gain)
        elif mode == 'dls':
            q_dot = dls_control(J, error, damping=damping, gain=gain)
        else:
            raise ValueError("mode must be 'pinv' or 'dls'.")

        q = q + q_dot * dt
        q_history.append(q.copy())
        qdot_history.append(q_dot.copy())

        print(
            f"Step {step:03d} | "
            f"pos={x} | "
            f"error_norm={error_norm:.6f} | "
            f"q_dot={q_dot}"
        )

    return {
        'final_q': q,
        'final_position': forward_kinematics(q, link_lengths),
        'positions': np.array(positions),
        'errors': np.array(errors),
        'q_history': np.array(q_history),
        'qdot_history': np.array(qdot_history),
    }


def main():
    print("\nRunning pseudoinverse controller:\n")
    pinv_results = run_controller(mode='pinv')

    print("\nRunning DLS controller:\n")
    dls_results = run_controller(mode='dls')

    print("\nFinal comparison:")
    print("Pseudoinverse final position:", pinv_results['final_position'])
    print("Pseudoinverse final error:", pinv_results['errors'][-1] if 	len(pinv_results['errors']) else "N/A")

    print("DLS final position:", dls_results['final_position'])
    print("DLS final error:", dls_results['errors'][-1] if len(dls_results['errors']) else"N/A")
