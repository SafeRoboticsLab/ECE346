# Create a private fork

1. Create a new **private** GitHub repo named ```ECE346_GroupXX``` following this [documentation](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-new-repository).

2. Open Terminal.

3. Create a bare clone of the repository.
    ```
    git clone --bare https://github.com/SafeRoboticsLab/ECE346.git
    ```

4. Mirror-push to the new repository.
    ```
    cd ECE346.git
    git push --mirror YOUR_NEW_REPO_URL
    ```

5. Remove the temporary local repository you created earlier.
    ```
    cd ..
    rm -rf ECE346.git
    ```

6. Clone your own private repo
    ```
    git clone YOUR_NEW_REPO_URL
    cd YOUR_NEW_REPO_NAME
    ```

5. Add the original ECE346 repo as remote to fetch future lab assignments and updates

    ```
    git remote add upstream https://github.com/SafeRoboticsLab/ECE346.git
    git remote set-url --push upstream DISABLE
    ```

# Update your fork from public repo
When you want to pull changes from upstream you can just fetch the remote and rebase on top of your work.
