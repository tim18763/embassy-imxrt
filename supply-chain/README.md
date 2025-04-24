# Working with cargo vet

## Introduction

`cargo vet` is a tool to help ensure that third-party Rust dependencies have been audited by a trusted entity.
It matches all dependencies against a set of audits conducted by the authors of the project or entities they trust.  
To learn more: [mozilla/cargo-vet](https://github.com/mozilla/cargo-vet)

## Adding a new dependency
If you're updating a dependency or adding a new one, you need to ensure it's been audited before being added to main.  
For our repositories, we have designated experts who are responsible for vetting any new dependencies being added to their repository.  
_It is the shared responsibility of the developer creating the PR and the auditors to conduct a successful audit._  
Please follow this process to ensure compliance:

- ### For developers
  - If your PR fails in the `cargo vet` step, the cargo-vet workflow will add a comment to the PR with a template questionnaire.
Copy the questionnaire and paste it as a new comment to the PR along with your answers. This greatly helps the auditors get some context of the changes requiring the new dependencies.  
  - Respond to any questions that the auditors might have regarding the need of any new dependencies.
  - Once the new audits have been merged into main by the auditors, rebase your branch on main, verify it passes `cargo vet`, and force push it
    ```bash
    git fetch upstream
    git rebase upstream/main
    cargo vet --locked
    git push -f
    ```
  - The existing PR comment from the previous failure will be updated with a success message once the check passes

- ### For auditors
  - Check the filled questionnaire on the PR once the developer responds to the `cargo vet` failure.
  - To audit new dependencies, inspect the `cargo vet` failures using your preferred method
    - Use [gh pr checkout](https://cli.github.com/manual/gh_pr_checkout) to checkout the PR and run `cargo vet --locked`
    - Use [Github Pull Requests for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=GitHub.vscode-pull-request-github) to checkout the PR and run `cargo vet --locked`
    - For more suggestions: [Checking out pull requests locally](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/checking-out-pull-requests-locally)
  - Follow the recommendations of the `cargo vet` command output, either `cargo vet diff` for version update or `cargo vet inspect` for new dependencies
  - Record the new audits using `cargo vet certify` to add them to _audits.toml_
  - Verify all dependencies are passing using `cargo vet`
  - Copy the updated _audits.toml_ to a new branch off main and submit the PR to update the audits
  - Mention the original PR# in the audits PR so it reflects in the original PR, making it easier for the developer to track the audits

  #### Tips:
  - Update _imports.lock_ to reduce number of audits by using `cargo vet` instead of `cargo vet --locked`
    - We can import trusted third party audits to reduce the number of audits we need to perform. Running `cargo vet` without `--locked` fetches new imports and updates _imports.lock_ with any audits that are helpful for our project.
  - If an audit cannot be performed for some dependency due to time sensitivity or business justified reasons, use `cargo vet add-exemption <PACKAGE> <VERSION>` to add the dependency to exemptions in _config.toml_
  - To add all remaining audits to exemptions at once, use `cargo vet regenerate exemptions`
  - Remove unnecessary exemptions and imports using `cargo vet prune`

<!-- TODO: Commented out until rust-crate-audits is private 
> Ideally, we want all new audits to be shared across ODP repositories to reduce the overhead of multiple audits for the same dependencies. To ensure audits are shared, it's recommended to cut and paste the audits as a separate PR to the _audits.toml_ in [rust-crate-audits](https://github.com/OpenDevicePartnership/rust-crate-audits).
> If due to business reasons, the audits are not to be shared across repositories, submit the audits to the _audits.toml_ in the project respository. -->