# fire-fighting-uav

## Development rules

Code cannot be pushed directly to the main branch on this repo. You will need to use pull requests in order to submit code to main and had it reviewed. Pull requests use git branches.

### Understanding git branches

Branches are useful tool how to structure code development. You can read more [here](https://www.atlassian.com/git/tutorials/using-branches) For the sake of pull requests, here is what you need to know.

**Creating new branch**

You can do this from your terminal as follows:

```
cd <your_git_repository>
git branch <name_of_branch>
```
Replace <your_git_repository> by the path to the folder holding your git repo code. Replace <name_of_branch> by the name you wanna use for your branch.

**Checking in which branch you are**

When working with branches, it is important to know in which branch you are, that you don't accidentally start to change code somewhere else. You can verify in which branch you are from terminal:

```
git branch -a
```
This command will list all available branches and the one, where you are currently, will be highlighted and have * in front of its name. 

**Change in which branch you are**
In order to change, in which branch you will be, run the following command:
```
git checkout <name_of_the_branch_you_wanna_go_in>
```

### How to use pull requests?

When you start developing a new feature, you will have to first create a new git branch. See above how to do this. **Importantly,** use a short but descriptive name as the branch name which captures the esence of the feature you are going to develop, e.g. "GNSSNavigation" would be a good name for a branch where you will add navigation using GNSS.

Then, develop your code as you are used to. When the feature is finished and tested, you can create a pull request from the github web interface. Open the repository, navigate to Pull requests tab, click on "New pull request". On top, you will see two expand menus. One which says "base: <something>" and the other which says "compare: <something>". Click on the expand button for the base and choose main (this is the branch where you want to merge your code into). Then, click on the expand button for compare and choose the name of your branch. When done, click on the button "Create pull request". 

Congrats! You have just made a pull request! Now, someone else needs to go and review it.

### Review of pull requests

When someone reviews your pull request, it is likely they will ask questions or ask you to make changes. You can do that by pushing another commit to the same branch you used for the development of this feature.

### Very Important Step for Onboard DJI SDK ROS installation

Change the original SDK node file and publisher file with the one from shival-data branch Onboard SDK files. The original files had several bugs which were sorted out with the DJI support centre and made it up & running with lab's M600Pro drone.
