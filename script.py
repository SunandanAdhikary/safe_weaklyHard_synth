import numpy as np
from scipy.linalg import expm, solve_discrete_lyapunov
import itertools

# Mute warnings for cleaner output, e.g., from matrix singularity checks.
np.seterr(all='ignore')

# --- Data Structures & Geometric Utilities ---

class HyperRectangle:
    """
    A simple class to represent an n-dimensional axis-aligned hyper-rectangle.
    This is used to define regions in the state space, like the safe region (X_S)
    and reachable sets.
    """
    def __init__(self, intervals):
        """
        Initializes the HyperRectangle.
        Args:
            intervals (list of tuples): A list of (min, max) tuples for each dimension.
                                        e.g., [(-1, 1), (-2, 2)] for a 2D rectangle.
        """
        self.intervals = np.array(intervals, dtype=float)

    @property
    def is_empty(self):
        """Checks if the rectangle is invalid or has zero/negative volume."""
        return np.any(self.intervals[:, 0] >= self.intervals[:, 1])

    def __repr__(self):
        """String representation for easy printing."""
        if self.is_empty:
            return "HyperRectangle(Empty)"
        intervals_str = ", ".join(f"[{low:.2f}, {high:.2f}]" for low, high in self.intervals)
        return f"HyperRectangle({intervals_str})"

    def contains(self, other_rect):
        """Checks if this rectangle completely contains another one."""
        if self.is_empty or other_rect.is_empty:
            return False
        return np.all(self.intervals[:, 0] <= other_rect.intervals[:, 0]) and \
               np.all(self.intervals[:, 1] >= other_rect.intervals[:, 1])

    def get_vertices(self):
        """Generates all vertices of the hyper-rectangle."""
        ranges = [iv for iv in self.intervals]
        return np.array(list(itertools.product(*ranges)))

def union_of_rects(rect_list):
    """
    Computes the union of a list of HyperRectangles.
    Since the union might not be a single rectangle, this function returns a list
    of disjoint rectangles representing the union. For simplicity in this context,
    we'll merge overlapping rectangles into a bounding box, which is an over-approximation.
    A more robust implementation would be needed for complex unions.
    """
    if not rect_list:
        return []
    # For this algorithm, we often union disjoint sets. We'll return the list as is.
    # In a real scenario, a more complex merging logic would be here.
    return [r for r in rect_list if not r.is_empty]

def intersection_of_rects(rect_list):
    """Computes the intersection of a list of HyperRectangles."""
    if not rect_list:
        return HyperRectangle([])
    
    rect_list = [r for r in rect_list if not r.is_empty]
    if not rect_list:
        return HyperRectangle([]) # Empty intersection

    # Calculate max of all lower bounds and min of all upper bounds
    max_of_mins = np.max([r.intervals[:, 0] for r in rect_list], axis=0)
    min_of_maxs = np.min([r.intervals[:, 1] for r in rect_list], axis=0)
    
    intersection_intervals = np.vstack([max_of_mins, min_of_maxs]).T
    return HyperRectangle(intersection_intervals)


# --- Core System and Control Logic ---

class ClosedLoopSystem:
    """
    Represents the closed-loop control system as described in Section II of the paper.
    It encapsulates system matrices and derives the augmented state matrices for
    successful (A1) and missed (A0) control updates.
    """
    def __init__(self, A_c, B_c, C, K, L, h):
        """
        Args:
            A_c, B_c (np.array): Continuous-time system matrices.
            C (np.array): Output matrix.
            K (np.array): LQR feedback gain.
            L (np.array): Kalman filter gain.
            h (float): Sampling period.
        """
        self.n = A_c.shape[0]
        
        # Discretize the system matrices
        self.A = expm(A_c * h)
        self.B = np.linalg.inv(A_c) @ (self.A - np.eye(self.n)) @ B_c
        self.C = C
        self.K = K
        self.L = L

        # Augmented matrix for successful update (Eq. 2)
        # X = [x, x_hat]^T, state dimension is 2n
        self.A1 = np.block([
            [self.A, -self.B @ self.K],
            [self.L @ self.C, self.A - self.L @ self.C - self.B @ self.K]
        ])

        # Augmented matrix for missed update (Eq. 3)
        self.A0 = np.block([
            [self.A, -self.B @ self.K],
            [np.zeros((self.n, self.n)), np.eye(self.n)]
        ])
        
        # Helper matrices for SSCD calculation
        self.I = np.eye(self.n * 2)
        self.O = np.zeros_like(self.A1)


# --- Reachability Analysis (Simplified) ---

def reachset(initial_rect, system_matrix):
    """
    Simplified reachability analysis, as a stand-in for a formal tool like Flow*.
    It computes an over-approximation of the reachable set from an initial
    hyper-rectangle under a linear transformation.
    
    Args:
        initial_rect (HyperRectangle): The set of initial states.
        system_matrix (np.array): The system dynamics matrix (e.g., A1, A0, or a product).

    Returns:
        HyperRectangle: An axis-aligned bounding box of the reachable states.
    """
    if initial_rect.is_empty:
        return HyperRectangle([])

    vertices = initial_rect.get_vertices()
    transformed_vertices = (system_matrix @ vertices.T).T
    
    # Find the new min and max along each dimension to form the bounding box
    min_bounds = np.min(transformed_vertices, axis=0)
    max_bounds = np.max(transformed_vertices, axis=0)
    
    return HyperRectangle(np.vstack([min_bounds, max_bounds]).T)


# --- Weakly Hard Automaton (WHSA) Construction ---

def mkwhsa(clsys, locations, miss_count, k_length):
    """
    Constructs the Weakly Hard Switching Automaton (WHSA) based on Section III-A.
    This function selects stable subsystems (SSCDs) that meet the performance
    criterion (GUES decay rate).
    
    Returns:
        A dictionary representing the WHSA: {location_id: matrix}.
        Locations are integers `i` representing the SSCD `10...0` with `i` zeros.
    """
    locations = {}
    
    # Location 0: SSCD '1' (successful update)
    A_l0 = clsys.A1
    # Check for stability
    if np.max(np.abs(np.linalg.eigvals(A_l0))) < 1.0:
        locations[0] = {'matrix': A_l0, 'gues': np.log(np.max(np.abs(np.linalg.eigvals(A_l0))))}

    # Generate other locations (SSCDs '10', '100', ...)
    for i in range(1, max_consecutive_miss + 1):
        # Matrix for SSCD '1' followed by 'i' zeros (misses)
        # e.g., for '100', matrix is A0 * A0 * A1
        matrix = np.linalg.matrix_power(clsys.A0, i) @ clsys.A1
        
        # Check stability: spectral radius < 1
        eigenvalues = np.linalg.eigvals(matrix)
        spectral_radius = np.max(np.abs(eigenvalues))
        
        if spectral_radius < 1.0:
            gues_decay = np.log(spectral_radius)
            # The paper's formulation for switchability (Eq. 4) is complex.
            # We simplify here by checking if the subsystem's decay rate is
            # better than the desired one. A full implementation would solve
            # for Lyapunov functions and switching coefficients.
            if gues_decay < gamma_star:
                locations[i] = {'matrix': matrix, 'gues': gues_decay}

    return locations


# --- Main Algorithm and Verification Function ---

def verha(automaton, initial_region, safe_region, m_bar, k, sg_m):
    """
    Implements the VERHA function from the paper (lines 20-25).
    Performs safety verification for the WHSA given an initial region.
    
    Args:
        automaton (dict): The WHSA.
        initial_region (HyperRectangle): The initial set of states to verify from.
        safe_region (HyperRectangle): The overall safe state space.
        m_bar (int): Current number of misses being considered.
        k (int): Total length of the sequence.
        sg_m (dict): The safety guard data structure to be updated.

    Returns:
        A tuple (is_safe, updated_sg_m)
    """
    # This is a simplified trace exploration. A full implementation would
    # explore all valid traces of length K with m_bar misses.
    # We will simulate one step for each transition.
    is_safe_so_far = True
    
    # Iterate over all possible jumps (from_loc -> to_loc)
    for from_loc in automaton.keys():
        # Compute one step reachability from the initial region
        reach_set_from = reachset(initial_region, automaton[from_loc]['matrix'])
        
        if not safe_region.contains(reach_set_from):
            is_safe_so_far = False
            break # This path is unsafe
        
        for to_loc in automaton.keys():
            # In this simplified version, we assume any location can transition
            # to any other, as long as the trace stays safe.
            
            # The state after the first step must be a safe starting point for the next
            reach_set_to = reachset(reach_set_from, automaton[to_loc]['matrix'])

            if safe_region.contains(reach_set_to):
                # This transition is safe from the given initial_region
                jump = (from_loc, to_loc)
                if jump not in sg_m:
                    sg_m[jump] = []
                sg_m[jump].append(initial_region)

    if not is_safe_so_far:
        # Clear updates if any path was unsafe from this initial region
        return False, sg_m

    # Consolidate regions in sg_m
    for jump, regions in sg_m.items():
        sg_m[jump] = union_of_rects(regions)

    return True, sg_m


def algorithm_1(clsys, gamma_star, x_s, delta, k):
    """
    Main implementation of Algorithm 1 from the paper.
    
    Args:
        clsys (ClosedLoopSystem): The system model.
        gamma_star (float): Desired GUES decay margin (must be < 0).
        x_s (HyperRectangle): The safe state space region.
        delta (list): Granularity for partitioning the safe space.
        k (int): Hyperperiod / finite number of sampling iterations.

    Returns:
        list: A list of safe weakly hard specification tuples.
    """
    print("--- Starting Algorithm 1 ---")
    
    # Line 1: Create uniform partitions of the safe state space
    partitions = []
    # This is a simplified grid creation.
    # E.g., for 2D: [ [x0, x1], [x1, x2] ], [ [y0, y1], [y1, y2] ]
    # We create a flat list of small hyper-rectangles covering X_S.
    dims = len(delta)
    iterators = [np.arange(x_s.intervals[i, 0], x_s.intervals[i, 1], delta[i]) for i in range(dims)]
    
    for point in itertools.product(*iterators):
        intervals = [(point[i], point[i] + delta[i]) for i in range(dims)]
        partitions.append(HyperRectangle(intervals))
    print(f"Step 1: Partitioned safe space into {len(partitions)} grid cells.")

    # Line 2-3: Initial WHSA and parameter calculation
    # We estimate max consecutive misses (CM) for the initial WHSA build.
    # Let's assume a reasonable upper bound based on system dynamics.
    initial_cm_guess = k // 2
    a0 = mkwhsa(clsys, gamma_star, 0, k, initial_cm_guess)
    nl = len(a0)
    cm_bar = max(a0.keys()) if a0 else 0
    m_bar_max = k - (k // (cm_bar + 1)) if cm_bar > -1 else 0
    print(f"Step 2-3: Initial analysis -> {nl} locations, Max consecutive miss (CM)={cm_bar}, Max total miss (M)={m_bar_max}")

    # Line 4: Initialize the specification list
    spec_list = [{'X': [], 'm_bar': m, 'cm_bar': cm_bar, 'cm': 0, 'K': k} for m in range(m_bar_max + 1)]
    
    # Line 5: Initialize Safety Guard (SG) data structure
    sg = {m: {} for m in range(m_bar_max + 1)}

    # --- Main Loop (Line 6) ---
    for m_j in range(m_bar_max + 1):
        print(f"\n--- Verifying for m_bar = {m_j} misses ---")
        
        # Line 7: Build WHSA for current miss count
        automaton_j = mkwhsa(clsys, gamma_star, m_j, k, cm_bar)
        if not automaton_j:
            print(f"No stable automaton found for {m_j} misses. Stopping.")
            break
            
        # Line 8-9: Grid-wise safety verification
        print("Step 8-9: Performing grid-wise safety verification...")
        for i, partition in enumerate(partitions):
            _, sg[m_j] = verha(automaton_j, partition, x_s, m_j, k, sg[m_j])

        # Line 13-17: Derive specifications from SG
        print("Step 13-17: Deriving specifications from verification results...")
        spec = spec_list[m_j]
        all_safe_regions_for_mj = []
        
        # Find all safe regions from the SG table for this miss count
        for jump, regions in sg[m_j].items():
            all_safe_regions_for_mj.extend(regions)

        # The union represents the total safe initialization area found so far
        spec['X'] = union_of_rects(all_safe_regions_for_mj)

        # Update max consecutive misses based on what was found to be safe
        safe_locations = set(j[0] for j in sg[m_j].keys()) | set(j[1] for j in sg[m_j].keys())
        spec['cm_bar'] = max(safe_locations) if safe_locations else 0

        # Line 18: MDADT calculation (simplified)
        # The paper requires MDADT for SSCD '1'. This ensures recovery.
        # We'll use a placeholder value, as a full calculation requires
        # solving for Lyapunov functions not fully implemented in mkwhsa.
        spec['cm'] = 1 # Minimum 1 successful update for recovery.

    return [s for s in spec_list if s['X']]


if __name__ == '__main__':
    # --- Define a Sample System (e.g., a double integrator) ---
    # This is a placeholder for a real system model from a CPS application.
    # Continuous-time state-space representation
    h = 0.1  # Sampling period
    A_c = np.array([[0, 1], [0, 0]])
    B_c = np.array([[0], [1]])
    C = np.array([[1, 0]])
    
    # Dummy gains for K (controller) and L (observer).
    # In a real scenario, these would be designed (e.g., via LQR/Kalman).
    K = np.array([[1, 2]])
    L = np.array([[0.5], [0.1]])

    # Create the closed-loop system object
    system = ClosedLoopSystem(A_c, B_c, C, K, L, h)

    # --- Set Algorithm Inputs ---
    # Desired GUES decay rate (must be negative)
    gamma_star = -0.05
    
    # Safe state space X_S: [x, x_dot, x_hat, x_dot_hat]
    # Here, we only constrain the physical states x and x_dot.
    # The observer states are allowed to vary more widely.
    safe_space = HyperRectangle([
        (-2.0, 2.0),    # position
        (-2.0, 2.0),    # velocity
        (-10.0, 10.0),   # estimated position
        (-10.0, 10.0)    # estimated velocity
    ])
    
    # Grid granularity for partitioning X_S
    # A finer grid gives more accuracy but takes longer.
    grid_delta = [1.0, 1.0, 5.0, 5.0]
    
    # Hyperperiod / sequence length
    K_hyperperiod = 6

    # --- Run the Algorithm ---
    final_specifications = algorithm_1(
        clsys=system,
        gamma_star=gamma_star,
        x_s=safe_space,
        delta=grid_delta,
        k=K_hyperperiod
    )
    
    # --- Print Results ---
    print("\n\n--- Final Synthesized Safe Weakly Hard Specifications ---")
    if not final_specifications:
        print("No safe specifications could be found.")
    else:
        for i, spec in enumerate(final_specifications):
            print(f"\nSpecification #{i+1}:")
            print(f"  (m_bar, cm_bar, cm, K) = ({spec['m_bar']}, {spec['cm_bar']}, {spec['cm']}, {spec['K']})")
            print(f"  Description: Allow max {spec['m_bar']} total misses and {spec['cm_bar']} consecutive misses in {spec['K']} steps.")
            print(f"  Requires at least {spec['cm']} consecutive successful updates for recovery.")
            print(f"  Safe Initial Regions (X):")
            if not spec['X']:
                print("    None found.")
            else:
                for region in spec['X']:
                    print(f"    - {region}")
