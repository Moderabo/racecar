import matplotlib.pyplot as plt

# Creates a grpah of the velocity in respect to time..
class Graph():

    def __init__(self, file):
        self.file = file
        self.time_vector = []
        self.vel_vector = []
        
        # Read the file with stored data.
        with open(self.file) as data_file:
            current_time = 0
            for line in data_file:
                words = line.strip("\n").split(" ")
                if words[0] == "Vel:":
                    self.vel_vector.append(float(words[2]))
                    self.time_vector.append(current_time)
                    current_time += float(words[1])
        
        # Plot velocity in respect to time. 
        plt.figure(self.file)
        plt.plot(self.time_vector, self.vel_vector)
        plt.xlabel("Tid [s]")
        plt.ylabel("Hastighet [mm/s]")
        plt.title("Hastighet av tävlingsbil.")
        plt.show()

        return