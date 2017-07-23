# KIT-ss2017-Praktikum
camera calibration
 calibration
 there are three parts in our code, they are area-scan camera calibration, line-scan camera calibration, stereo camera calibration.
 we have put these 3 parts in one file.
 
 how to use the code:
 
     clear all
     
    for area-scan camera calibration:
    
    1.take pitures and input them in Matlab
    2.use toolbox to calibrate pictures and export camera parameters.
    
    //the file 'calibrationSession.mat' includes all parameters of area-scan camera we have got after first 'Aufnahme'
   
    
    for line-scan camera calibration:
    
    1.read the picture c-z5.png under directory /picture/zeilen
    or use line-scan camera to take a new picture
    2. use grae scale value picture to choose a suitabal 'K' value
    3. set 'K' value in programm
    
    
    for stereo camera calibration:
    run the programm to calculate the translation distance between area-scan camera and line-scan camera
    
