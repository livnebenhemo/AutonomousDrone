* Clone the GitHub repo
```
git clone git@github.com:rbdlabhaifa/AutonomousDroneCPP.git
```
* Install Opencv with the following link: https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/

Make sure you do git checkout 3.4.16 in opencv and opencv_contrib.

you can check if the checkout worked with 

```
git describe --tags
```

Afterwards, you need to install the Eigen library.

Once you've done that, run
```
chmod +x installDep.sh
./installDep
```
 If there are problems with flip.cpp in ctello/examples, change add "include" to the ctello.h in the file.
