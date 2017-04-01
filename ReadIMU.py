import wiringpi2
import time
import math

#Obnizona rozdzielczosc pomiaru, w razie potrzeby zmodyfikowac rejestry

gyro_address = 0x6b;
acc_address = 0x1d;

gyro_reg1 = 0b00100000;
gyro_reg1_start = 0b00001111;
gyro_reg4 = 0b00100011;
gyro_reg4_start = 0b00110000;

acc_reg1 = 0b00100000;
acc_reg1_start = 0b01010111;
acc_reg4 = 0b00100011;
acc_reg4_start = 0b00101000;

gyro_x_angle = 0;
gyro_y_angle = 0;
gyro_z_angle = 0;

loop_time = 0.01;
gyro_sens = 0.0175;
dt=0;
prev_time=0;


def start_dev (address, adres_rejestru1, adres_rejestru2, bajt_danych1, bajt_danych2):

    fd = wiringpi2.wiringPiI2CSetup(address);

    wiringpi2.wiringPiI2CWriteReg8(fd, adres_rejestru1, bajt_danych1);
    wiringpi2.wiringPiI2CWriteReg8(fd, adres_rejestru2, bajt_danych2);
    time.sleep(0.1);
    return fd;

#inicjalizacja zyroskopu
id = start_dev(gyro_address, gyro_reg1, gyro_reg4, gyro_reg1_start, gyro_reg4_start);

#inicjalizacja akcelerometru
id_a = start_dev(acc_address, acc_reg1, acc_reg4, acc_reg1_start, acc_reg4_start);


#------------------------------------kalibracja---------------------------------------
cx=[0]*51;
cx2=0;
cx3=0;

cy=[0]*51;
cy2=0;
cy3=0;

cz=[0]*51;
cz2=0;
cz3=0;

acx=[0]*51;
acx2=0;
acx3=0;

acy=[0]*51;
acy2=0;
acy3=0;

acz=[0]*51;
acz2=0;
acz3=0;

def calibrate():

    for v in range(0,50):

      LSBx = wiringpi2.wiringPiI2CReadReg16(id, 0x22);
      MSBx = wiringpi2.wiringPiI2CReadReg16(id, 0x23);
      if MSBx & 0x8000:
         MSBx = -(0x010000 - MSBx)
      x = ((MSBx << 8) | LSBx);


      LSBy = wiringpi2.wiringPiI2CReadReg16(id, 0x24);
      MSBy = wiringpi2.wiringPiI2CReadReg16(id, 0x25);
      if MSBy & 0x8000:
          MSBy = -(0x010000 - MSBy)
      y = ((MSBy << 8) | LSBy);

      LSBz = wiringpi2.wiringPiI2CReadReg16(id, 0x26);
      MSBz = wiringpi2.wiringPiI2CReadReg16(id, 0x27);
      if MSBz & 0x8000:
         MSBz = -(0x010000 - MSBz)
      z = ((MSBz << 8) | LSBz);

      gyro_rate_x = x * gyro_sens;
      cx[v]=gyro_rate_x;
      global cx2;
      cx2 += cx[v];

      gyro_rate_y = y * gyro_sens;
      cy[v]=gyro_rate_y;
      global cy2;
      cy2 += cy[v];


      gyro_rate_z = z * gyro_sens;
      cz[v]=gyro_rate_z;
      global cz2;
      cz2 += cz[v];

def acc_calibrate():

    for t in range(0,50):

      LSBx = wiringpi2.wiringPiI2CReadReg16(id_a, 0x28);
      MSBx = wiringpi2.wiringPiI2CReadReg16(id_a, 0x29);
      acc_x = ((MSBx << 8) | LSBx) >> 4;

      LSBy = wiringpi2.wiringPiI2CReadReg16(id_a, 0x2A);
      MSBy = wiringpi2.wiringPiI2CReadReg16(id_a, 0x2B);
      acc_y = ((MSBy << 8) | LSBy) >> 4;

      LSBz = wiringpi2.wiringPiI2CReadReg16(id_a, 0x2C);
      MSBz = wiringpi2.wiringPiI2CReadReg16(id_a, 0x2D);
      acc_z = ((MSBz << 8) | LSBz) >> 4;

      acc_x_angle = (math.atan2(acc_y,acc_z)+3.141592653589793238546)*57.29578
      acx[t]=acc_x_angle;
      global acx2;
      acx2 += acx[t];

      acc_y_angle = (math.atan2(acc_x,acc_z)+3.141592653589793238546)*57.29578
      acy[t]=acc_y_angle;
      global acy2;
      acy2 += acy[t];

      acc_z_angle = (math.atan2(acc_x,acc_y)+3.141592653589793238546)*57.29578
      acz[t]=acc_z_angle;
      global acz2;
      acz2 += acz[t];

# Strojenie filtra

Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03

class Filter:

    # Deklaracja stalych

    P = [[0 for o in range(2)] for p in range(2)]

    staticAngle = 0
    staticRate = 0
    angleDiff = 0
    bias = 0

    S = 0
    K = [0 for o in range(2)]


    def kalman(self, angle, rate, dt):

        """
        global staticAngle
        global staticRate
        global P
        global bias
        global angleDiff
        """

        self.staticRate = rate - self.bias
        self.staticAngle += self.staticRate * dt

        # Macierz kowariancji, predykcja dryftu

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += Q_bias *dt

        # Estymacja bledu, wzmocnienie Kalmana

        self.S = self.P[0][0] + R_measure
        self.K[0] = self.P[0][0] / self.S
        self.K[1] = self.P[1][0] / self.S

        self.angleDiff = angle - self.staticAngle
        self.staticAngle += self.K[0] * self.angleDiff
        self.bias += self.K[1] * self.angleDiff

        # Aktualizacja macierzy kowariancji

        self.P[0][0] -= self.K[0] * self.P[0][0]
        self.P[0][1] -= self.K[0] * self.P[0][1]
        self.P[1][0] -= self.K[1] * self.P[0][0]
        self.P[1][1] -= self.K[1] * self.P[0][1]

        return self.staticAngle


#----------------------------------odczyt z zyroskopu---------------------------------

calibrate();
calibrate_x = cx2/51;
calibrate_y = cy2/51;
calibrate_z = cz2/51;

acc_calibrate();
acc_calibrate_x = acx2/51;
acc_calibrate_y = acy2/51;
acc_calibrate_z = acz2/51;

x_angle = 0;
y_angle = 0;
z_angle = 0;

p=0;

# Filter class object initialization

x_axis = Filter()
y_axis = Filter()
z_axis = Filter()


while 1:

 if (time.time() - prev_time > loop_time):
   prev_time = time.time()

   #osX
   LSBx = wiringpi2.wiringPiI2CReadReg16(id, 0x22);
   MSBx = wiringpi2.wiringPiI2CReadReg16(id, 0x23);
   if MSBx & 0x8000:
      MSBx = -(0x010000 - MSBx)
   gyro_x = ((MSBx << 8) | LSBx);

   gyro_rate_x = gyro_x * gyro_sens;
   gyro_x_time = (gyro_rate_x- calibrate_x)*dt/50;
   gyro_x_angle += gyro_x_time;

   #print "Os X = ", gyro_x_angle;

   #osY
   LSBy = wiringpi2.wiringPiI2CReadReg16(id, 0x24);
   MSBy = wiringpi2.wiringPiI2CReadReg16(id, 0x25);
   if MSBy & 0x8000:
      MSBy = -(0x010000 - MSBy)
   gyro_y = ((MSBy << 8) | LSBy);

   gyro_rate_y = gyro_y * gyro_sens;
   gyro_y_time = (gyro_rate_y - calibrate_y)*dt/50;
   gyro_y_angle += gyro_y_time;

   #print "Os Y = ", gyro_y_angle;

   #osZ
   LSBz = wiringpi2.wiringPiI2CReadReg16(id, 0x26);
   MSBz = wiringpi2.wiringPiI2CReadReg16(id, 0x27);
   if MSBz & 0x8000:
      MSBz = -(0x010000 - MSBz)

   gyro_z = ((MSBz << 8) | LSBz);

   gyro_rate_z = gyro_z * gyro_sens;
   gyro_z_time = (gyro_rate_z - calibrate_z)*dt/50;
   gyro_z_angle += gyro_z_time;

   #print "Os Z = ", gyro_z_angle;



#----------------------------------odczyt z akcelerometru---------------------------------


   #osX
   LSBx = wiringpi2.wiringPiI2CReadReg16(id_a, 0x28);
   MSBx = wiringpi2.wiringPiI2CReadReg16(id_a, 0x29);
   if MSBx & 0x8000:
      MSBx = -(0x010000 - MSBx)
   acc_x = ((MSBx << 8) | LSBx) >> 4;


   #osY
   LSBy = wiringpi2.wiringPiI2CReadReg16(id_a, 0x2A);
   MSBy = wiringpi2.wiringPiI2CReadReg16(id_a, 0x2B);
   if MSBy & 0x8000:
      MSBy = -(0x010000 - MSBy)
   acc_y = ((MSBy << 8) | LSBy) >> 4;


   #osZ
   LSBz = wiringpi2.wiringPiI2CReadReg16(id_a, 0x2C);
   MSBz = wiringpi2.wiringPiI2CReadReg16(id_a, 0x2D);
   if MSBz & 0x8000:
     MSBz = -(0x010000 - MSBz)
   acc_z = ((MSBz << 8) | LSBz) >> 4;


   acc_x_angle = (math.atan2((acc_y),(acc_z))+3.141592653589793238546)*57.29578
   #if acc_x_angle > 180:
      #acc_x_angle -= 180
   #print "Os X = ", (acc_x_angle - acc_calibrate_x);

   acc_y_angle = (math.atan2((acc_x),(acc_z))+3.141592653589793238546)*57.29578
   #if acc_y_angle > 180:
      #acc_y_angle -= 180
   #print "Os Y = ", (acc_y_angle - acc_calibrate_y);

   acc_z_angle = (math.atan2((acc_x),(acc_y))+3.141592653589793238546)*57.29578
   #if acc_z_angle > 180:
      #acc_z_angle -= 180
   #print "Os Z = ", (acc_z_angle - acc_calibrate_z);

   # === Dane po filtracji, przechowywane w osobnych obiektach ====

   print(str(x_axis.kalman(acc_x_angle, gyro_x_angle, dt)) + '     ' + str(y_axis.kalman(acc_y_angle, gyro_y_angle, dt)) + '     ' + str(z_axis.kalman(acc_z_angle, gyro_z_angle, dt)))

   if p<1000:
      osX = open("/home/pi/Desktop/osX.txt", "a")
      osX.write(str(gyro_x_angle) + '     ' + str(acc_x_angle - acc_calibrate_x) + '\n')
      osX.close()

      osY = open("/home/pi/Desktop/osY.txt", "a")
      osY.write(str(gyro_y_angle) + '     ' + str(acc_y_angle - acc_calibrate_y) + '\n')
      osY.close()
   elif p==1000:
      print('-------------KONIEC SERII POMIAROWEJ---------------')

   p+=1;
   time.sleep(0.01);
   dt = time.time() - prev_time;




