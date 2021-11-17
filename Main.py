import environment

# 初始化
ev = environment.Highway(figure1=1,figure2=1)
episodes = 50000
score_list = []

# 每个回合运行
for i in range(episodes):
    step = 1000
    score = 0
    s = ev.reset()

    # 每步运行
    for j in range(step):
        next_s, reward, done = ev.run(0)
        if done:
            score_list.append(score)
            break
