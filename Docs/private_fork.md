# Create a private fork

**If you have never used git before, you may want to check out this introductory [tutorial](https://www.atlassian.com/git/tutorials).**

1. Create a new **private** GitHub repo named ```ECE346_GroupXX``` (you can follow the steps [here](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-new-repository)).

2. Open a Terminal window on your computer.

3. If you haven't already, create a clone of the `ECE346` repository on your machine. Go to the cloned directory.
    ```
    git clone --recurse-submodules https://github.com/SafeRoboticsLab/ECE346.git
    cd ECE346
    ```

4. From inside the cloned directory, rename the original `ECE346` GitHub repo from `origin` to `upstream`, to fetch future lab assignments and updates.
    ```
    git remote rename origin upstream
    git remote set-url --push upstream DISABLE
    ```
5. Set your freshly created private repo as the new `origin` using the URL assigned to it on GitHub. The URL of your private repo can be found on the main page of your GitHub repository (online).  After you navigate to the main page of your GitHub repo, select the green <>Code icon.  Under the Clone category, select SSH and copy this URL.  (NOTE: At this point, you should have already setup your GitHub SSH Key.  If you didn't, this step will not work.)
    ```
    git remote add origin <URL of your private Repo>
    ```
4. Push the `SP2023` branch from your local clone repo to your new remote one, which has now become a private fork of `ECE346`.
    ```
    git push -u origin SP2023
    ```
    
# Push to your private repo
When woking on the labs and making changes to your code, you can push the code to your private repo on GitHub by simply doing:
```bash
git push origin
```    
# Update your fork from the original public repo
## First, *commit all of your changes* you have made to your private epo.
**Not sure about merge? It is never a bad idea to keep a copy locally before merging.**

1. Create a temporary local branch on your computer.
    ```
    git checkout -b temp
    ```
2. You can now merge the original `upstream` repo into your local branch.
    ```
    git pull upstream SP2023 
    ```
    This will create a merge commit for you. If you encounter any conflicts, this [tutorial](https://www.atlassian.com/git/tutorials/using-branches/merge-conflicts) can help you take care of them.
3. Inspect all changes that you have made in the temporary branch, then checkout your *SP2023* branch.
    ```
    git checkout SP2023
    git merge temp
    git branch –-delete temp
    # Update submodules in case there are any
    git submodule update --init --recursive
    ```
Once you are fully comfortable with the git merge workflow, you may want to skip steps 1 and 3 and pull directly into your local `SP2023` branch.
