# Mathematical Foundation of RRT* (Optimal RRT)

The RRT* (Rapidly-exploring Random Tree Star) algorithm is an extension of RRT that provides **asymptotic optimality**, ensuring that the generated path converges to the shortest possible path as the number of iterations increases.

## Core Operations

### 1. Sampling
A random state $q_{rand}$ is sampled from the configuration space $Q_{free}$.  Goal biasing is applied with a probability $p_{bias}$:
$$P(q_{rand} = q_{goal}) = p_{bias}$$

### 2. Nearest Neighbor Search
Find the nearest node $q_{near}$ in the existing tree $T$ using the Euclidean distance $d(q_1, q_2)$:
$$q_{near} = \text{argmin}_{q \in T} d(q, q_{rand})$$

### 3. Steering
Define a new node $q_{new}$ by moving from $q_{near}$ towards $q_{rand}$ by a fixed step size $\epsilon$:
$$q_{new} = q_{near} + \epsilon \cdot \frac{q_{rand} - q_{near}}{||q_{rand} - q_{near}||}$$

### 4. Optimal Parent Selection
Search for all nodes $Q_{near} \subseteq T$ within a ball of radius $r(n) = \gamma \sqrt{\frac{\ln(n)}{n}}$ centered at $q_{new}$. Choose $q_{parent}$ that minimizes the total cost from the root:
$$q_{parent} = \text{argmin}_{q \in Q_{near}} \{ \text{cost}(q) + d(q, q_{new}) \}$$

### 5. Rewiring
For each $q \in Q_{near} \setminus \{q_{parent}\}$, update its parent to $q_{new}$ if it reduces its total path cost:
$$\text{If } \text{cost}(q_{new}) + d(q_{new}, q) < \text{cost}(q): parent(q) \leftarrow q_{new}$$

## Implementation Details
The algorithm utilizes a dynamic search radius that shrinks as the tree grows, maintaining efficiency while ensuring optimality.
