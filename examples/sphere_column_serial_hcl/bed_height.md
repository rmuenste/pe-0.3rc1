If we account for the **security zone $\rho$** (which acts as an effective diameter increase) and look at **Figure 4**, the numbers align perfectly with your observation of $\approx 12$ inches.

Here is the corrected derivation:

### 1. The Effective Particle Size
Because the repulsive force activates at separation $\rho$, the particles must be initialized with a center-to-center distance **at least** equal to $d + \rho$ to avoid infinite forces at $t=0$.

*   **Diameter ($d$):** $0.635 \text{ cm}$
*   **Security Zone ($\rho$):** $0.06858 \text{ cm}$
*   **Effective Pitch ($d_{eff}$):**
    $$ d_{eff} = d + \rho = 0.70358 \text{ cm} $$

### 2. Recalculating the "Square Lattice"
Now we determine how many "effective spheres" fit in the box width.

*   **Bed Width ($W$):** $20.30 \text{ cm}$
*   **Particles per Row:**
    $$ N_{row} = \frac{W}{d_{eff}} = \frac{20.30}{0.70358} \approx 28.85 $$
    
    Since they must fit without overlap, the maximum integer number of particles per row is **28** (down from my previous estimate of 32).

### 3. Calculating the Height
With fewer particles per row, the stack must be taller to fit all 1204 spheres.

*   **Number of Rows:**
    $$ N_{rows} = \frac{1204 \text{ particles}}{28 \text{ particles/row}} = \mathbf{43 \text{ rows}} $$

*   **Initial Bed Height ($H_0$):**
    $$ H_0 = N_{rows} \times d_{eff} = 43 \times 0.70358 \text{ cm} \approx \mathbf{30.25 \text{ cm}} $$

### 4. Verification with Figure 4
Let's convert that height to inches to match the plot.

$$ H_0 \text{ (in)} = \frac{30.25 \text{ cm}}{2.54 \text{ cm/in}} \approx \mathbf{11.91 \text{ inches}} $$

**Comparison:**
Looking at **Figure 4 (Curve a)** for $V=3 \text{ cm s}^{-1}$:
*   The curve starts at $t=0$.
*   The intercept is almost exactly on the grid line for **12 inches**.
