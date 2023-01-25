# Create a private fork

**If you have never used git before, please check out this [tutorial](https://www.atlassian.com/git/tutorials).**

1. Create a new **private** GitHub repo named ```ECE346_GroupXX``` following this [documentation](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-new-repository).

2. Open Terminal.

3. Create a clone of the repository.
    ```
    git clone https://github.com/SafeRoboticsLab/ECE346.git
    ```
    **Remember that you will also need to initialize and update your submodule by**
    ```bash
    cd ECE346
    git submodule update --init --recursive
    ```
4. Rename the original ECE346 repo as upstream to fetch future lab assignments and updates
    ```
    git remote rename origin upstream
    git remote set-url --push upstream DISABLE
    ```
5. Set your private repo as the new origin
    ```
    git remote add origin PRIVATE_REPO_URL
    ```
4. Push from your local repo to your new remote one.
    ```
    git push -u origin Spring2022
    ```
    
# Push to your private repo
When woking on the lab and making changes to your code, you can push the code to your private repo by
```bash
git push origin
```    
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



