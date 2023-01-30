import gym
import numpy as np
import matplotlib.pyplot as plt

passo = np.array([0.25, 0.25, 0.01, 0.1])

def DiscretizeState(state):
    ds = state/passo
    ds = ds + np.array([15,10,1,10])
    return tuple(ds.astype(int))

env = gym.make('CartPole-v1')

QDim = [30, 30, 50, 50]
Q = np.zeros(QDim + [env.action_space.n])
episodes = 30000
mod = 50

epRwds = []
alpha = 0.1 #taxa de aprendizagem
gamma = 0.9 #fator de desconto
epsilon = 0.6 #probabilidade de ação aleatória

for i in range(episodes):
    ds = DiscretizeState(env.reset())
    done = False
    epRw = 0
    t = 0
    
    while not done:
        if i % mod == 0:
            action = np.argmax(Q[ds])
            ns, r, done, _ = env.step(action)
            epRw += r
            nds = DiscretizeState(ns)
        else:
            if np.random.random() > epsilon:
                action = np.random.randint(0, 2)
            else:
                action = np.argmax(Q[ds])

            ns, r, done, _ = env.step(action)
            nds = DiscretizeState(ns)

            Q[ds + (action,)] = Q[ds + (action,)] + alpha * (r + gamma * np.max(Q[nds]) - Q[ds + (action,)])

        ds = nds
        t += 1
    
    if i % mod == 0:
        #print("Episode finished after {} timesteps".format(t+1))
        epRwds.append(epRw)

ds = DiscretizeState(env.reset())
done = False
while not done:
    env.render()
    action = np.argmax(Q[ds])
    ns, r, done, _ = env.step(action)
    epRw += r
    nds = DiscretizeState(ns)

data = []
size = 10
for i in range (int(len(epRwds)/size)):
    summ = 0
    for j in range (size):
        summ += epRwds[i*size + j]
    mean = summ/size
    data.append(mean)

#print(np.arange(0, len(data)*1000, 1000))

plt.plot(np.arange(0, len(data)*500, 500), data , c = 'green')
plt.ylabel("Recompensa Acumulada")
plt.xlabel("Episódios")
plt.show()

env.close()
