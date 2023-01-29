# Create a private fork

**If you have never used git before, please check out this [tutorial](https://www.atlassian.com/git/tutorials).**

1. Create a new **private** GitHub repo named ```ECE346_GroupXX``` following this [documentation](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-new-repository).

2. Open Terminal.

3. Go the `ECE346` folder that you have already cloned. If you have not done that, create a clone of the repository.
    ```
    git clone --recurse-submodules https://github.com/SafeRoboticsLab/ECE346.git 
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
    git push -u origin SP2023
    ```
    
# Push to your private repo
When woking on the lab and making changes to your code, you can push the code to your private repo by
```bash
git push origin
```    
# Update your fork from public repo
## First, *commit all of your changes* you have made to your private epo. Not sure about merge? It is never a bad idea to keep a copy locally before merge.

1. Create a temporary branch
    ```
    git checkout -b temp
    ```
2. You can now merge the upstream 
    ```
    git pull upstream SP2023 
    ```
    This will create a merge commit for you. If you encounter any conflicts, please checkout this [tutorial](https://www.atlassian.com/git/tutorials/using-branches/merge-conflicts).
3. Inspect all changes that you have made in the temporary branch, then checkout your *SP2023* branch.
    ```
    git checkout SP2023
    git merge temp
    git branch –delete temp
    ```
    



