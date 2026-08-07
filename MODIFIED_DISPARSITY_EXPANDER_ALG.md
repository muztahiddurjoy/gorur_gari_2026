The **Modified Disparity Extender** algorithm drives the car using 4 quick steps:

1. **Expands Car Width (Safety Bubble):** 
   Instead of thin laser lines, it gives the car an invisible safety bubble ($13\text{cm} \to 16\text{cm}$, expanding with speed) and shortens any path where the car's body would hit a wall or pillar.

2. **Detects Pillars ($5\text{cm}$ Width):** 
   Spots sudden distance jumps in LiDAR data, calculates the object's physical size ($s = r \cdot \theta$), and points the camera servo at confirmed pillars to read their color.

3. **Enforces WRO Rules (Red/Green Override):** 
   Blocks invalid paths around pillars—forcing the car to steer **Left of Green towers** and **Right of Red towers**.

4. **Steers & Accelerates:** 
   Picks the deepest remaining safe path, adjusts the steering servo, and boosts speed on clear straights!