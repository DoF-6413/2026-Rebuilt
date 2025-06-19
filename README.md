# 2026-Rebuilt
Code Base for FRC Team 6413's 2026 Rebuilt Robot Code

## Code Etiquette
  USEFUL comments on EVERYTHING (Commands, Constants, etc)
  Organize files properly in corresponding folders
  Follow naming conventions (reference section below)
  No hardcoded values, add them to Constants
  **NEVER** merge code into the main or dev branches that you know does not work!
  
  Create Issues on Github for EVERY branch
  - Add a description of what the branch should accomplish
  - 'Assignees': used to indicate who is working on what
  - 'Labels': used to organize issues, add as necessary
  - 'Development': link the branch to the issue

  LOG OUT when finished for the day
  - Use the **deactivate_ssh_key** script to remove your SSH key from active use.
  - Log out of Slack, GitHub, Google, and anything else

   Beta Numbers
 - first number = issue #
 - second number = # of commits
 - third number = functionality (0 = works as intended, 1 = WIP, 2 = doesn't work)

For example, on the branch associated with issue 7, with 18 commits, which is still in development should look like: "7.18.1"

<hr/>

## Branch Organization

Branch names MUST be all lower case with spaces turned into hyphens
  - Ex: feat#69-shooter-interpolation, not feat#69-ShooterInterpolation

main:

- Competition-ready code
- ONLY receives merges from dev
- Do **NOT** make branches off of main

dev:

- **Make ALL branches off of *this* branch!!**
- Code on dev should **always** be runnable.  
- Only merge to main after tested and cleaned
  - Explanatory comments
  - Code is fully functional

feat#[Issue#]-[name]:

- Used for new features on the robot or updates to existing code
  - Ex: feat#40-arm or feat#68-mechanism-button-bindings
  - Only contains what the branch is named
    - Ex: No climber code in a shooter branch
-  Gets merged to dev when code is tested and clean (see "dev" for "tested and clean" standards)

bugfix#[Issue#]-[name]:

- Used for bug fixing of a feature from dev

chore#[Issue#]-[name]:

- Used for cleaning code from dev
- Should NOT be necessary if you always follow our coding etiquette and standards

## Naming Conventions

- Folder names are all lowercase and underscores instead of spaces
  - Ex: subsystems or commands\zero_commands
- File and Java Class names are in Pascal Case: the first letter of every word is capitalized and no spaces between words.
  - Ex: RobotContainer.java or public class RobotContainer 
- Constants are in all caps with underscores between words
  - Ex: public static final ELEVATOR_MAX_CURRENT = 5.0 
- Functions and variables are in camelCase: lowercase first word, capitalize first letter of all subsequent words, no spaces between words
  - Ex: public void updateModeChange() 

<hr/>

## Folder/File Organization

- Root level files
  - Constants.java
    - Constants shared by all subsystems, related to the robot
    - Ex: controller port numbers, battery voltage, alliance (red/blue), etc.
  - Main.java
    - Runs Robot
  - Robot.java
    - Initializes robot, starts logging data, runs periodic/auto/simulation modes
  - RobotContainer.java
    - Creates all subsystems, runs commands, and logs data
- **commands**
  - **teleop_commands**
    - Commands used for Teleop
    - Ex: scoring XXX, intaking game pice from a specific location, etc.
  - **zero_commands**
    - Commands to reset all subsystems / mechanisms to their "zero" positions
- **subsystems**
  - Ex: arm, drive, vision, etc.
  - [subsystem]
    - [Subsystem.java]: main class for subsystem, runs commands depending on passed in IO (sim or real), extends SubsystemBase
    - [Subsystem]IO.java: interface for inputs and methods
    - [Subsystem]IOSim.java: simulation code for subsystem, implements [Subsystem]IO
    - [Subsystem]IO[Motor].java: real code for subsystem, implements [Subsystem]IO
- **utils**
  - PathPlanner.java
    - Used in autos to follow a path
  - PIDController.java
    - Custom DoF PID Controller class
  - PoseEstimator.java
    - Calculates pose based on sensors

<hr/>

## Getting started on any SSPF laptop

Bfore you can use any Git commands or do any coding on an SSPF latpop you **MUST** run the **activate_ssh_key** script with your first name.  For example, if John wanted to code she would simply open a new Command Prompt on the laptop and type:

C:\User\SSPFUser> **activate_ssh_key John**

John would then be prompted to enter his SSH passphrase in order to confirm to the SSH Agent that the key he is the SSH keys owner.  Once done John's SSH key would be in use, Git would be configured to reflect he is the person making all changes and he would be put into her own home directory on the laptop (e.g. C:\SSPFUser\john_doe).  

If there was a problem with the SSH key activation process, the script will output an error message telling you what went wrong and what to do next.

We recommend that you clone the GitHub repo into its own folder in your personal folder on the laptop.  The repo can be in the users home directory (e.g. C:\SSPFUser\john_doe\2026-Rebuilt) or in a folder under the home directory (e.g. C:\SSPFUser\john_doe\repos\2026-Rebuilt)

<hr/>

## Useful Git Bash Commands

- git status
  - Shows you essential information about the current state of your branch.  Use this to see what unsaved changes there are and other info about your branch.
- git add .
  - Stages **all** changed files in preparation for commit and push.  Use git status before you do this to ensure you only commit the changs you expect and want.
- git commit -m "[Insert Message Here]"
  - Saves your code changes locally with a msg of what was done to the code.  Your message should be brief and informative, NOT snarky or unhelpful.
- git push
  - Pushes any local saved changes to the cloud
- git fetch
  - Pulls down any changes to your branch from the cloud.  Use this before you use git push to avoid possible merge problems.  If changes are pulled down then you should use git merge to merge them into your code before you do anything else.
- git pull
  - Pulls down any changes to your branch from the cloud and merges them into your local code.
  - git pull does a git fetch followed by a git merge
- git pull origin dev
  - Pulls changes from the dev branch in the cloud repository
- git merge (origin) Dev
  - Pulls ALL changes from dev (no rebases)
- git checkout [branch name]
  - Changes your branch
- git checkout -b [branch name]
  - Creates new branch off of current branch.  You SHOULD only be doing this when currently on the dev branch and you just did a git pull!
- git branch
  - Shows all branches that have been pulled on laptop
- git branch --all
  - Shows all branches since last fetch
