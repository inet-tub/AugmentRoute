import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_excel('Final res.xlsx', engine='openpyxl')

NumeOfValues = 15
Augmentation = [1, 1.05, 1.07, 1.1, 1.12, 1.14, 1.15, 1.2, 1.3, 1.4, 1.5 , 1.6, 1.75, 1.85, 2] 

Validity = [0]*NumeOfValues
Rounds = [0]*NumeOfValues
ValidityPer = [0]*NumeOfValues
AvgRounds = [0]*NumeOfValues

countAll = 243
for i in range(243):
    print(data[0][i])
    if(data[1][i] == 1):
        countAll-=1

for k in range(len(Augmentation)):
    for i in range(243):
            if (data[k+1][i] > 1):
                Validity[k]+=1
                Rounds[k]+=data[k+1][i]
    AvgRounds[k] = Rounds[k]/Validity[k]
    ValidityPer[k] = Validity[k]/countAll*100
    print(Augmentation[k], ValidityPer[k], AvgRounds[k])

            
plt.rcParams.update({'font.size': 22})

fig, ax1 = plt.subplots()
myColor = '#4F89C4'
ax1.set_xlabel('Multiplicative Augmentation')
ax1.set_xticks(np.arange(1, 2.2, 0.2))
ax1.set_ylabel('Average Number of Rounds', color=myColor)
ax1.set_ylim([2,8])
ax1.set_yticks(np.arange(2, 8, 1))
ax1.plot(Augmentation, AvgRounds, color=myColor, linewidth=4)
ax1.tick_params(axis='y')
ax1.grid(None, axis='both', linestyle='--')

ax2 = ax1.twinx() 
myColor = '#9C2F6D'
ax2.set_ylabel('Percentage of Instances', color=myColor) 
ax2.set_ylim([0,120])
ax2.set_yticks(np.arange(0, 110, 20))
ax2.plot(Augmentation, ValidityPer, color=myColor, linewidth=4)
ax2.tick_params(axis='y')
fig.tight_layout()
plt.subplots_adjust(wspace=0.5)
plt.show()
plt.savefig("IntroRealData", bbox_inches='tight')
