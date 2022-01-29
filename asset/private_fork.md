# Create a private fork

**If you have never used git before, please check out this [tutorial](https://www.atlassian.com/git/tutorials).**

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

6. Clone your own private repo and initialize the submodule
    ```
    git clone YOUR_NEW_REPO_URL
    cd YOUR_NEW_REPO_NAME
    git submodule update --init --recursive
    ```

5. Add the original ECE346 repo as remote to fetch future lab assignments and updates

    ```
    git remote add upstream https://github.com/SafeRoboticsLab/ECE346.git
    git remote set-url --push upstream DISABLE
    ```

# Push to your private repo
    git push origin
    
# Update your fork from public repo
## First, commit all of your changes you have made to your private epo. Not sure about merge? It is never a bad idea to keep a copy locally before merge.

1. (Optional) When you want to pull changes from upstream you can first fetch the upstream remote and inspect the difference between the public repo (upstream/Spring2022) and your own repo by
    ```
    git fetch upstream
    git diff upstream/Spring2022 Spring2022
    ```
2. You can now merge the upstream 
    ```
    git pull upstream Spring2022 
    ```
    This will create a merge commit for you. If you encounter any conflicts, please checkout this [tutorial](https://www.atlassian.com/git/tutorials/using-branches/merge-conflicts).



