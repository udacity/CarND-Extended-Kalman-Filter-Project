## Xcode IDE profile

Use this profile to edit the project using Xcode.

To use this profile, open `ExtendedKF.xcodeproj` in this folder.

To build using Xcode, re-generate this project as follows:

1. Remove `ExtendedKF.xcodeproj` file from this directory
2. Execute the following command in the current directory: `cmake -G "Xcode" ../..`

Steps above are required because cmake's Xcode generator uses absolute paths that will not work on your machine.
