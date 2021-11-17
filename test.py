import Environment
import numpy as np
from tensorflow.keras import models

# 初始化
model = models.load_model('DQN_Test_1109.h5')
ev = Environment.Highway(figure1=1,figure2=1)
s = ev.reset()
score = 0
step = 1000
for i in range(step):
    a = np.argmax(model.predict(np.array([s]))[0])
    s, reward, done = ev.run(a)
    score += reward
    if done:
        break
print('score:', score)

