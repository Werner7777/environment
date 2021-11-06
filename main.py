import tensorflow as tf
import pandas as pd
import matplotlib.pyplot as plt
import Environment
import ReinforceLearning

# 初始化
ev = Environment.Highway(figure1=1,figure2=1)
agent = ReinforceLearning.DQN()
episodes = 50000
score_list = []

# 每个回合运行
for i in range(episodes):
    step = 1000
    score = 0
    s = ev.reset()
    # 每步运行
    for j in range(step):
        a = agent.act(s)
        next_s, reward, done = ev.run(a)
        agent.remember(s, a, next_s, reward)
        agent.train()
        score += reward
        s = next_s
        if done:
            score_list.append(score)
            print('episode:', i, 'score:', score, 'max:', max(score_list))
            break

# 训练完成 画一个总的走势图