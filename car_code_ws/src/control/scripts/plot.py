import matplotlib.pyplot as plt
import numpy as np

def storage_vector(error_state_seq,error_state,true_state_seq,true_state,c_seq,cc,input_seq,error_input,true_input,input,i,ref_seq,ref):
    # storage of vectors
    error_state_seq = np.append(error_state_seq,error_state).reshape(i+2,3)

    true_state_seq = np.append(true_state_seq,true_state).reshape(i+2,3)

    ref_seq = np.append(ref_seq,ref).reshape(i+2,3)

    c_seq = np.append(c_seq,cc).reshape(i+1,10)
    
    input_seq = np.append(input_seq,error_input).reshape(i+1,2)

    true_input = np.append(true_input,input).reshape(i+1,2)

    return error_state_seq,error_state,true_state_seq,true_state,c_seq,cc,input_seq,error_input,true_input,input,ref_seq

def figure_plot(n,T,true_state_seq1,true_input1,true_state_seq2,true_input2,true_state_seq3,true_input3,true_state_seq4,true_input4,true_state_seq5,true_input5,ref_seq1,ref_seq2,ref_seq3,ref_seq4,ref_seq5):
    # make data
    time = np.linspace(0, n*T, n)
    # 画图
    fig = plt.figure()
    # plot
    plt.xlabel('x',fontsize=20)
    plt.ylabel('y',fontsize=20)
    plt.grid(True)
    plt.plot(true_state_seq1[:,0], true_state_seq1[:,1], linewidth=2.0)
    plt.plot(true_state_seq2[:,0], true_state_seq2[:,1], linewidth=2.0)
    plt.plot(true_state_seq3[:,0], true_state_seq3[:,1], linewidth=2.0)
    plt.plot(true_state_seq4[:,0], true_state_seq4[:,1], linewidth=2.0)
    plt.plot(true_state_seq5[:,0], true_state_seq5[:,1], linewidth=2.0)
    plt.plot(true_state_seq1[180,0],true_state_seq1[180,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq2[180,0],true_state_seq2[180,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq3[180,0],true_state_seq3[180,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq4[180,0],true_state_seq4[180,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq5[180,0],true_state_seq5[180,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq1[470,0],true_state_seq1[470,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq2[470,0],true_state_seq2[470,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq3[470,0],true_state_seq3[470,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq4[470,0],true_state_seq4[470,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq5[470,0],true_state_seq5[470,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq1[500,0],true_state_seq1[500,1], marker='o', markersize=10, color='green')
    plt.plot(true_state_seq2[500,0],true_state_seq2[500,1], marker='o', markersize=10, color='green')
    plt.plot(true_state_seq3[500,0],true_state_seq3[500,1], marker='o', markersize=10, color='green')
    plt.plot(true_state_seq4[500,0],true_state_seq4[500,1], marker='o', markersize=10, color='green')
    plt.plot(true_state_seq5[500,0],true_state_seq5[500,1], marker='o', markersize=10, color='green')
    plt.plot(true_state_seq1[550,0],true_state_seq1[550,1], marker='o', markersize=10, color='blue')
    plt.plot(true_state_seq2[550,0],true_state_seq2[550,1], marker='o', markersize=10, color='blue')
    plt.plot(true_state_seq3[550,0],true_state_seq3[550,1], marker='o', markersize=10, color='blue')
    plt.plot(true_state_seq4[550,0],true_state_seq4[550,1], marker='o', markersize=10, color='blue')
    plt.plot(true_state_seq5[550,0],true_state_seq5[550,1], marker='o', markersize=10, color='blue')
    plt.plot(true_state_seq1[600,0],true_state_seq1[600,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq2[600,0],true_state_seq2[600,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq3[600,0],true_state_seq3[600,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq4[600,0],true_state_seq4[600,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq5[600,0],true_state_seq5[600,1], marker='o', markersize=10, color='black')
    plt.plot(true_state_seq1[800,0],true_state_seq1[800,1], marker='o', markersize=10, color='blue')
    plt.plot(true_state_seq2[800,0],true_state_seq2[800,1], marker='o', markersize=10, color='blue')
    plt.plot(true_state_seq3[800,0],true_state_seq3[800,1], marker='o', markersize=10, color='blue')
    plt.plot(true_state_seq4[800,0],true_state_seq4[800,1], marker='o', markersize=10, color='blue')
    plt.plot(true_state_seq5[800,0],true_state_seq5[800,1], marker='o', markersize=10, color='blue')

    fig = plt.figure()
    # plot
    plt.xlabel('time/s',fontsize=20)
    plt.ylabel('x',fontsize=20)
    plt.grid(True)
    plt.plot(time, true_state_seq1[0:n,0], linewidth=2.0)
    plt.plot(time, true_state_seq2[0:n,0], linewidth=2.0)
    plt.plot(time, true_state_seq3[0:n,0], linewidth=2.0)
    plt.plot(time, true_state_seq4[0:n,0], linewidth=2.0)
    plt.plot(time, true_state_seq5[0:n,0], linewidth=2.0)

    fig = plt.figure()
    # plot
    plt.xlabel('time/s',fontsize=20)
    plt.ylabel('y',fontsize=20)
    plt.grid(True)
    plt.plot(time, true_state_seq1[0:n,1], linewidth=2.0)
    plt.plot(time, true_state_seq2[0:n,1], linewidth=2.0)
    plt.plot(time, true_state_seq3[0:n,1], linewidth=2.0)
    plt.plot(time, true_state_seq4[0:n,1], linewidth=2.0)
    plt.plot(time, true_state_seq5[0:n,1], linewidth=2.0)

    fig = plt.figure()
    # plot
    plt.xlabel('time/s',fontsize=20)
    plt.ylabel('phi',fontsize=20)
    plt.grid(True)
    plt.plot(time, true_state_seq1[0:n,2], linewidth=2.0)
    plt.plot(time, true_state_seq2[0:n,2], linewidth=2.0)
    plt.plot(time, true_state_seq3[0:n,2], linewidth=2.0)
    plt.plot(time, true_state_seq4[0:n,2], linewidth=2.0)
    plt.plot(time, true_state_seq5[0:n,2], linewidth=2.0)

    fig = plt.figure()
    # plot
    plt.xlabel('time/s',fontsize=20)
    plt.ylabel('Velocity',fontsize=20)
    plt.grid(True)
    plt.plot(time, true_input1[:,0], linewidth=2.0)
    plt.plot(time, true_input2[:,0], linewidth=2.0)
    plt.plot(time, true_input3[:,0], linewidth=2.0)
    plt.plot(time, true_input4[:,0], linewidth=2.0)
    plt.plot(time, true_input5[:,0], linewidth=2.0)


    fig = plt.figure()
    # plot
    plt.xlabel('time/s',fontsize=20)
    plt.ylabel('Angular Velocity',fontsize=20)
    plt.grid(True)
    plt.plot(time, true_input1[:,1], linewidth=2.0)
    plt.plot(time, true_input2[:,1], linewidth=2.0)
    plt.plot(time, true_input3[:,1], linewidth=2.0)
    plt.plot(time, true_input4[:,1], linewidth=2.0)
    plt.plot(time, true_input5[:,1], linewidth=2.0)

    fig = plt.figure()
    # plot
    plt.xlabel('x',fontsize=20)
    plt.ylabel('y',fontsize=20)
    plt.grid(True)
    plt.plot(ref_seq1[:,0], ref_seq1[:,1], linewidth=2.0)
    plt.plot(ref_seq2[:,0], ref_seq2[:,1], linewidth=2.0)
    plt.plot(ref_seq3[:,0], ref_seq3[:,1], linewidth=2.0)
    plt.plot(ref_seq4[:,0], ref_seq4[:,1], linewidth=2.0)
    plt.plot(ref_seq5[:,0], ref_seq5[:,1], linewidth=2.0)


    plt.show()