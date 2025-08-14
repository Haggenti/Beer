import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

def exponential_func(x, a, b, c):
    """Exponential function y = a * e^(bx) + c"""
    return a * np.exp(b * x) + c

def get_data_points():
    """Gets data points from the user"""
    n_points = int(input("How many measurement points? "))
    
    temperatures = []
    pwm_values = []
    
    for i in range(n_points):
        print(f"\nPoint {i+1}:")
        temp = float(input("Temperature (°C): "))
        pwm = float(input("PWM maintain (%): "))
        temperatures.append(temp)
        pwm_values.append(pwm)
    
    return np.array(temperatures), np.array(pwm_values)

def main():
    print("Coefficient calculator for the maintenance curve")
    print("Equation: PWM = A * e^(B*Temp) + C")
    print("---------------------------------------------------")
    
    # Get data
    temperatures, pwm_values = get_data_points()
    
    # Calculate coefficients by regression
    try:
        popt, _ = curve_fit(exponential_func, temperatures, pwm_values, 
                           p0=[0.01, 0.08, 6.0],  # Initial values close to those in the code
                           bounds=([0, 0, 0], [1, 1, 20]))  # Parameter limits
        
        a_fit, b_fit, c_fit = popt
        
        print("\nCalculated coefficients:")
        print(f"A = {a_fit:.5f}")
        print(f"B = {b_fit:.5f}")
        print(f"C = {c_fit:.4f}")
        
        # Create the graph
        plt.figure(figsize=(10, 6))
        
        # Data points
        plt.scatter(temperatures, pwm_values, color='blue', label='Measured points')
        
        # Fitted curve
        temp_curve = np.linspace(min(temperatures), max(temperatures), 100)
        pwm_curve = exponential_func(temp_curve, a_fit, b_fit, c_fit)
        plt.plot(temp_curve, pwm_curve, 'r-', label='Fitted curve')
        
        plt.xlabel('Temperature (°C)')
        plt.ylabel('PWM maintain (%)')
        plt.title('Temperature maintenance curve')
        plt.grid(True)
        plt.legend()
        plt.show()
        
    except RuntimeError:
        print("\nError: Unable to find satisfactory coefficients.")
        print("Try with other measurement points.")

if __name__ == "__main__":
    main()