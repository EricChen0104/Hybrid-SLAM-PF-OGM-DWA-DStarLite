# Contributing to Hybrid-SLAM-PF-OGM-DWA-DStarLite

We welcome contributions to the `Hybrid-SLAM-PF-OGM-DWA-DStarLite` project! Whether you're reporting a bug, suggesting an enhancement, or contributing code, your help is valuable.

This project implements a hybrid SLAM approach combining Particle Filter (PF) for localization and Occupancy Grid Map (OGM) for mapping, integrated with Dynamic Window Approach (DWA) for local planning and D* Lite for global path planning in a simulated environment.

Please read this document carefully before making any contributions.

## Table of Contents

1.  [Code of Conduct](#1-code-of-conduct)
2.  [How to Contribute](#2-how-to-contribute)
    *   [Reporting Bugs](#reporting-bugs)
    *   [Suggesting Enhancements](#suggesting-enhancements)
    *   [Code Contributions](#code-contributions)
3.  [Development Setup](#3-development-setup)
4.  [Coding Guidelines](#4-coding-guidelines)
5.  [Pull Request Process](#5-pull-request-process)
6.  [Contact](#6-contact)

---

## 1. Code of Conduct

We are committed to providing a welcoming and inclusive environment for everyone. Please treat all contributors and users with respect and kindness. Harassment and discrimination are not tolerated.

## 2. How to Contribute

### Reporting Bugs

If you find a bug in the simulation or code, please help us by reporting it. Before opening a new issue, please:

*   **Search existing issues**: Your bug might have already been reported.
*   **Provide detailed information**:
    *   A clear and concise description of the bug.
    *   Steps to reproduce the behavior (e.g., "Run the simulation, click at X, Y, then Z happens").
    *   Expected behavior.
    *   Actual behavior.
    *   Screenshots or animated GIFs (if applicable) can be very helpful.
    *   Your operating system and Python version.

### Suggesting Enhancements

We're always looking for ways to improve the `Hybrid-SLAM-PF-OGM-DWA-DStarLite` simulation. If you have an idea for a new feature or an improvement to existing functionality:

*   **Search existing issues**: Your suggestion might already be discussed.
*   **Describe your idea clearly**: Explain what the enhancement is, why it's useful, and how it might be implemented.

### Code Contributions

We welcome code contributions! Here's a general workflow:

1.  **Fork** the repository to your GitHub account.
2.  **Clone** your forked repository to your local machine:
    ```bash
    git clone https://github.com/YOUR_USERNAME/Hybrid-SLAM-PF-OGM-DWA-DStarLite.git
    cd Hybrid-SLAM-PF-OGM-DWA-DStarLite
    ```
3.  **Create a new branch** for your feature or bug fix:
    ```bash
    git checkout -b feature/your-feature-name # For new features
    # OR
    git checkout -b bugfix/your-bug-fix      # For bug fixes
    ```
4.  **Make your changes**.
5.  **Commit your changes** with clear and concise commit messages (see [Coding Guidelines](#4-coding-guidelines)).
6.  **Push your branch** to your forked repository:
    ```bash
    git push origin feature/your-feature-name
    ```
7.  **Open a Pull Request** (PR) to the `main` branch of the original repository.

## 3. Development Setup

To get the project running locally:

### Prerequisites

*   Python 3.x
*   Required Python packages:
    *   `numpy`
    *   `matplotlib`
    *   `scipy`

You can install these using pip:

```bash
 
pip install numpy matplotlib scipy
 
```

### Running the Simulation
1. Save the provided code as PF_OGM_DWA_D_STAR_LITE.py.
2. Navigate to the directory containing the file in your terminal.
3. Run the simulation using Python:
```bash
 
python PF_OGM_DWA_D_STAR_LITE.py
 
```
4. Interaction during runtime:
 - The simulation window will open with two plots.
 - Left-click on the left plot (SLAM visualization) to set a new goal for the robot. The robot will then begin planning and moving towards this goal.
 - Right-click on the left plot to add a new temporary circular obstacle at that location in the simulated environment. The robot's planner will dynamically react to these.
 - Close the simulation window to stop the program.

## 4. Coding Guidelines
To maintain code quality and consistency across the project:
- Style: Adhere to PEP 8 for Python code style.
- Clarity: Write clear, readable code with meaningful variable and function names.
- Comments: Use comments to explain complex logic, non-obvious choices, or sections of code that might not be immediately clear.
- Docstrings: Consider adding docstrings to new functions and classes to explain their purpose, arguments, and return values.
- Functionality: Ensure your changes do not break existing functionality. Test your changes thoroughly by running the simulation and verifying expected behavior.
- Commit Messages: Write clear, concise, and descriptive commit messages. A good commit message explains what changed and why.

## 5. Pull Request Process
When submitting a Pull Request:
- One feature/bug per PR: Each PR should address a single feature or bug fix to keep reviews focused.
- Clear Description: Provide a detailed description of your changes. Explain what was changed, why it was changed, and any impact it might have. Reference any relevant issues (e.g., Fixes #123, Closes #45).
- Visuals: If your changes involve visual aspects of the simulation or demonstrate new behavior, include screenshots or GIFs in your PR description.
- Review: Your PR will be reviewed by the maintainer(s). Be open to feedback and willing to make adjustments based on the review.

## 6. Contact
If you have any questions or need further clarification, feel free to open an issue on the GitHub repository.
