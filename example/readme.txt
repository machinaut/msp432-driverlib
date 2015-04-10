The following steps allow you to debug driverlib code located in ROM.

1.	Exclude driverlib.c from your build (already done for you in the example project)
2.	Launch Debug session 
3.	Run > Load > Remove All Symbols
4.	Run > Load > Add Symbol > driverlib.out first
5.	Run > Load > Add Symbol > your project.out
6. 	Begin debugging your code. When the application is in a ROM API CCS will prompt you to point to the driverlib.c file
7.	Now you can debug ROM
