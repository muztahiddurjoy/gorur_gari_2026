Here are the exact rules for the WRO 2026 Future Engineers documentation and GitHub repository. The documentation is worth up to 30 points (approximately 25% of your total score) and serves as the primary tiebreaker.

### 1. General Submission Requirements

* **Format:** You must maintain a public GitHub repository and submit a printed hardcopy at the international final.


* **Language:** All information and documentation on GitHub must be in English for the international competition.


* **Availability:** The GitHub link must be provided no later than three weeks before the competition. The repository must be set to public when submitted and remain public for at least 12 months after the event.


* **Reproducibility:** A core goal of the documentation is to provide enough detail so that another team could completely reproduce your robot.



### 2. Required GitHub Content

Your repository must contain the following specific elements:

* **Engineering Journal:** A discussion and motivation for the vehicle's mobility, power and sense architectures, and obstacle management strategies.


* **Media:** Photos of the vehicle from every side, top, and bottom, along with a team photo.


* **Video:** A URL to a YouTube video (public or unlisted) showing the vehicle driving autonomously. You must provide one video for each challenge, and the driving demonstration portion must be at least 30 seconds long.


* **Source Code & CAD:** The code for all programmed components, as well as any files used for 3D printers, laser cutting, or CNC machines. All code must be well documented with comments.



### 3. The `README.md` File

* The repository must contain a `README.md` file with a description of the designed solution that is **not less than 5,000 characters**.


* It must clarify the code modules, how they relate to the electromechanical components, and provide step-by-step instructions on the process to build, compile, and upload the code to the controllers.



### 4. Strict Commit Deadlines

The history of your GitHub repository must show a progression of your engineering process, requiring at least three specific commits:

* **Commit 1:** Must be made **not later than 2 months before the competition** and must contain no less than 1/5 (20%) of the final amount of code.


* **Commit 2:** Must be made **not later than 1 month before the competition**.


* **Commit 3:** Must be made **not later than 2 weeks before the competition**. *Note: Judges will primarily use this commit for evaluation and scoring; changes made after this point might not be scored*.



### 5. Scoring Criteria (The 30 Points)

Judges score your documentation across five criteria, awarding 0, 2, 4, or 6 points for each. To get the maximum 6 points (Advanced Engineering), you must explicitly justify your design choices and tradeoffs.

1. **Mobility and Mechanical Design:** Includes torque/speed reasoning, design tradeoffs, and evidence of testing/iterations that improved mechanical performance.


2. **Power and Sensor Architecture:** Includes a power budget, sensor tradeoffs, placement justified by field geometry, calibration methods, and failure point considerations.


3. **Software Architecture and Obstacle Strategy:** Includes state machine rationale, justified algorithms (e.g., PID, CV methods), edge case handling, and documented testing metrics.


4. **Systems Thinking and Engineering Decisions:** Explicitly identifies constraints, risks, and failure modes. You must include reasoning based on data/tests (e.g., "we chose X instead of Y because...").


5. **Reproducibility and GitHub Quality:** The robot is fully reproducible, the project structure is clear, commit messages are meaningful, and the testing workflow is documented.