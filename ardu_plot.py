import matplotlib.pyplot as plt
import serial
import time

ser = serial.Serial(port="COM3", baudrate=115200)

def read_img():
    img = []

    has_start = False

    while(True):
        line = ser.readline().decode("utf-8").strip()
        print("Processing: ", line)
        if line == '**START':
            has_start = True
            continue

        if has_start and line == '**END':
            break
        
        if has_start:
            row = []
            for i in line.split():
                row.append(int(i))
            img.append(row)

    return img

def display(plot, img):
    plt.imshow(img, cmap="gray")
    plt.show()

if __name__ == "__main__":
    fig = None
    while(True):
        img = read_img()
        if fig is None:
            fig = plt.imshow(img, cmap="gray")
            # fig.show(block=False)
        else:
            fig.set_data(img)
            # fig.canvas.draw_idle()

        plt.pause(0.01)
