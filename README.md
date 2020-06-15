# Portable cargo volume measurement system

Sponsored by Panasonic

## Background
In recent years, with the development of mobile Internet and the strong support of national policies, China's e-commerce market has grown. The increase in the number of online transactions in a geometric series has brought huge business opportunities to the logistics industry. But there are many new challenges. How to optimize every steps of express delivery, so that the goods can be efficient and safe Delivered to consumers? There are some problems including slow speed, tedious manual input, error in freight calculation, and abnormal measurement when using the traditional manual measurement method for the volume measurement of deliver goods. In order to solve these problems, saving enterprise labor and time costs, standardizing the measurement process, and improving efficiency. It has become more and more practical to develop a portable cargo volume measurement system.

## Purpose
Based on the dual camera, a portable cargo volume measurement system has been developed, which can accurately measure the volume of the cuboid cargo resting on the flat ground.

System parameter requirements:

1. Cargo shape: cuboid
2. Minimum size of cargo: 5cm * 5cm * 5cm 
3. Maximum size of cargo: 70cm * 70cm * 70cm 
4. Cargo position: still on flat ground
5. Camera: a TOF sensor plus an ordinary RGB sensor 
6. Camera position: Handhold movement 
7. Time-consuming: no more than 500ms
8. Network status: running offline

## Expected Deliverable
1. Indoor environment, accurately measure the side length of the goods, the unilateral error is less than 3 ‰. 
2. Indoor environment, accurately measure the volume of the goods, the error is less than 1%.

## Notes
1. ColorSpec (Color Specification) https://www.mathworks.com/help/matlab/ref/colorspec.html