# DarkBox
This is the Arduino code for the DarkBox test fixture.

If you have questions, contact a.nutt7@gmail.com or marcusfinlinson@gmail.com

The code is designed to be easily extensible to new products with different brightness and turn-on threshold requirements.
To add a new product, do the following:
1. Find the list of setting #defines at the top of the file and add your state name. Make sure that you define it to be the next number in ascending order.
For example, if you want to add the state SUPER_BRIGHT, add the following (Assuming the previous one was assigned to be 2):
#define SUPER_BRIGHT 3

2. Copy the block of text below and append a prefix to uniquely identify the values. Set the numbers to be the correct thresholds for your project.

#define BRIGHTNESS_MIN 20
#define BRIGHTNESS_MAX 30
#define THRESHOLD_MIN 1
#define THRESHOLD_MAX 5

3. Modify the switch statements using the light_setting variable to include your setting. Those are contained in the functions isBrightnessGood(), isThresholdGood(), and drawScreen().
Copy and paste another setting's statements, and then modify the values to correspond to the desired values for the new product.
