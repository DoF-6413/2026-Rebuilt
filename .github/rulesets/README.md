# Branch rulesets

GitHub does not read rulesets from files in the repo. The JSON here is the
reviewed definition of the rule; a repo admin has to import it for it to
take effect.

## protect-default-branch.json

- Blocks direct pushes to the default branch: changes must come through a
  pull request.
- Requires 1 approval, and that approval must come from a code owner.
  `.github/CODEOWNERS` names `@DoF-6413/mentors`, so a mentor must approve.
- Blocks force pushes and deletion of the default branch.
- No one is on the bypass list.

## Applying it

1. Merge the `.github/CODEOWNERS` change first. Without it the code owner
   requirement has nothing to enforce and any writer's approval is enough.
2. Repo Settings -> Rules -> Rulesets -> New ruleset -> Import a ruleset.
3. Choose `protect-default-branch.json` and create it.

If the rule is changed later in Settings, export it and update this file.
