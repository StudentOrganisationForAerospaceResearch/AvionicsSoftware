# IDE Directories Folder
## About
Support for IDEs other STM32CubeIDE

Any new IDEs added must have all relevant .gitignores added to the .gitignore in the root.


## IDE Setup
### [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html#get-software)
* Download and install CubeIDE, allow driver installation (this is necessary to use ST-Link)
* Launch STM32CubeIDE, it will ask for a workspace. Setup the workspace <b>anywhere you'd like</b> (it doesn't matter), I would recommend in the root directory above where you cloned / will clone the repo.
* In CubeIDE select "Import Projects" then "Existing Projects into Workspace" and select the root directory **above** where AvionicsSoftware is cloned.
* Select **Finish**

### [Visual Studio 2022 (Windows Only)](https://visualstudio.microsoft.com/vs/)
* Install VS2022 (no extensions required yet, as VS2022 is currently inactive for build)
* Open [AvionicsSoftware.sln](https://github.com/StudentOrganisationForAerospaceResearch/AvionicsSoftware/tree/Chris/OuroborosC%2B%2B/_IDE/VS2022)
