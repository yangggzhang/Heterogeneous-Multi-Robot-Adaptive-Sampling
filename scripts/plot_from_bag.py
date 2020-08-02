import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys

experiment_num = 15
scenario_num = 1

def bag_info_to_list(bag, info):
	info_list=[]
	for topic, msg, t in bag.read_messages(topics=['/sampling_performance']):
		if info == "sample_count":
			info_list.append(msg.sample_count)
		elif info == "rmse":
			info_list.append(msg.rmse)
		elif info == "average_variance":
			info_list.append(msg.average_variance)
		else:
			raise NameError('msg name not correct. Try sample_count, rmse, average_variance')
	return info_list

def get_info_from_data_by_type_all_experiments(bag_name, length, scenario_num, info_type):
	total_info = []
	for i in range(1,scenario_num+1): #scenario 1 indexed
		for j in range(experiment_num):
			rosbag_file = bag_name+"scenario"+str(i)+"_"+str(j)+".bag"
			bag  = rosbag.Bag(rosbag_file)
			info_list = bag_info_to_list(bag, info_type)
			if not (len(info_list)<length):
				total_info.append(info_list[:length])
	total_info = np.array(total_info)
	return total_info

def get_info_from_data_by_type_single_scenario(bag_name, length, scenario_id, info_type):
	total_info = []
	for j in range(experiment_num):
		rosbag_file = bag_name+"scenario"+str(scenario_id)+"_"+str(j)+".bag"
		bag  = rosbag.Bag(rosbag_file)
		info_list = bag_info_to_list(bag, info_type)
		if not (len(info_list)<length):
			total_info.append(info_list[:length])
	total_info = np.array(total_info)
	return total_info

def get_min_length_for_plotting_all_experiments(bag_name, scenario_num):
	min_length = sys.maxint
	for i in range(0,scenario_num): #scenario 1 indexed
		for j in range(experiment_num):
			rosbag_file = bag_name+"scenario"+str(i)+"_"+str(j)+".bag"
			bag  = rosbag.Bag(rosbag_file)
			min_length = min(min_length, bag.get_message_count('/sampling_performance'))
	return min_length

def main():
	# check min length of trial
	hetero_length = get_min_length_for_plotting_all_experiments("./hetero/", scenario_num)
	homo_length = get_min_length_for_plotting_all_experiments("./homo/", scenario_num)
	length = min(hetero_length, homo_length)
	#----------------------------------------#
	# Get mean and std over all experiments  #
	#----------------------------------------#

	homo_total_average_variance = get_info_from_data_by_type_all_experiments("./homo/", length, scenario_num, 'average_variance')
	print(homo_total_average_variance.shape)
	homo_mean_average_variance = homo_total_average_variance.mean(axis=0)
	homo_std_average_variance = homo_total_average_variance.std(axis=0)
	homo_t = np.arange(homo_total_average_variance.shape[1])
	print(homo_t.shape)
	print(homo_mean_average_variance.shape)

	# scenario_num = 1
	hetero_total_average_variance = get_info_from_data_by_type_all_experiments("./hetero/", length, scenario_num, 'average_variance')
	print(hetero_total_average_variance.shape)
	hetero_mean_average_variance = hetero_total_average_variance.mean(axis=0)
	hetero_std_average_variance = hetero_total_average_variance.std(axis=0)
	hetero_t = np.arange(hetero_total_average_variance.shape[1])
	print(hetero_t.shape)
	print(hetero_mean_average_variance.shape)


	# scenario_num = 2
	homo_total_rms = get_info_from_data_by_type_all_experiments("./homo/", length, scenario_num, 'rmse')
	print(homo_total_rms.shape)
	homo_mean_rms = homo_total_rms.mean(axis=0)
	homo_std_rms = homo_total_rms.std(axis=0)
	homo_t = np.arange(homo_total_rms.shape[1])
	print(homo_t.shape)
	print(homo_mean_rms.shape)

	# scenario_num = 1
	hetero_total_rms = get_info_from_data_by_type_all_experiments("./hetero/", length, scenario_num, 'rmse')
	print(hetero_total_rms.shape)
	hetero_mean_rms = hetero_total_rms.mean(axis=0)
	hetero_std_rms = hetero_total_rms.std(axis=0)
	hetero_t = np.arange(hetero_total_rms.shape[1])
	print(hetero_t.shape)
	print(hetero_mean_rms.shape)

	# scenario_num = 2
	homo_total_num_samples = get_info_from_data_by_type_all_experiments("./homo/", length, scenario_num, 'sample_count')
	print(homo_total_num_samples.shape)
	homo_mean_num_samples = homo_total_num_samples.mean(axis=0)
	homo_std_num_samples = homo_total_num_samples.std(axis=0)
	homo_t = np.arange(homo_total_num_samples.shape[1])
	print(homo_t.shape)
	print(homo_mean_num_samples.shape)

	# scenario_num = 1
	hetero_total_num_samples = get_info_from_data_by_type_all_experiments("./hetero/", length, scenario_num, 'sample_count')
	print(hetero_total_num_samples.shape)
	hetero_mean_num_samples = hetero_total_num_samples.mean(axis=0)
	hetero_std_num_samples = hetero_total_num_samples.std(axis=0)
	hetero_t = np.arange(hetero_total_num_samples.shape[1])
	print(hetero_t.shape)
	# print(hetero_mean_num_samples.shape)

	# scenario_num = 2
	# homo_total_avg_variance = get_info_from_data_by_type_all_experiments("./homo/", length, scenario_num, 'average_variance')
	# print(homo_total_avg_variance.shape)
	# homo_mean_avg_variance = homo_total_num_samples.mean(axis=0)
	# homo_std_avg_variance = homo_total_num_samples.std(axis=0)
	# homo_t = np.arange(homo_total_avg_variance.shape[1])
	# print(homo_t.shape)

	# scenario_num = 1
	# hetero_total_avg_variance = get_info_from_data_by_type_all_experiments("./hetero/", length, scenario_num, 'average_variance')
	# print(hetero_total_avg_variance.shape)
	# hetero_mean_avg_variance = hetero_total_avg_variance.mean(axis=0)
	# hetero_std_avg_variance = hetero_total_avg_variance.std(axis=0)
	# hetero_t = np.arange(hetero_total_avg_variance.shape[1])
	# print(hetero_t.shape)

	#------------------------------------#
	#             Plot Result            #
	#------------------------------------#	
	plt.rcParams.update({'font.size': 30})
	plt.rcParams['pdf.fonttype']=42
	fig, ax = plt.subplots(1)
	line1 = ax.plot(homo_t, homo_mean_rms,lw=2, label="homogeneous RMS error", color = 'blue')
	ax.fill_between(homo_t, homo_mean_rms+homo_std_rms, homo_mean_rms-homo_std_rms, facecolor='blue',alpha=0.1)

	line2 = ax.plot(hetero_t, hetero_mean_rms,lw=2, label="heterogeneous RMS error", color = 'red')
	ax.fill_between(hetero_t, hetero_mean_rms+hetero_std_rms, hetero_mean_rms-hetero_std_rms, facecolor='red',alpha=0.1)
	ax.set_ylabel('RMS Error (uW)', size=35)
	ax.set_xlabel('ROS Time (s)',size=35)
	ax.legend(loc=0)
	ax2 = ax.twinx()
	line3 = ax2.plot(homo_t, homo_mean_num_samples,lw=2, label="homogeneous number of samples", color = 'blue',linestyle="--")
	ax2.fill_between(homo_t, homo_mean_num_samples+homo_std_num_samples, homo_mean_num_samples-homo_std_num_samples, facecolor='blue',alpha=0.1)
	# plt.axvline(x=25, color="maroon", lw=2.5)
	# plt.text(15, -33, 'Ariel Robot \nBattery Died',size=26)
	line4 = ax2.plot(hetero_t, hetero_mean_num_samples,lw=2, label="heterogeneous number of samples", color = 'red',linestyle="--")
	ax2.fill_between(hetero_t, hetero_mean_num_samples+hetero_std_num_samples, hetero_mean_num_samples-hetero_std_num_samples, facecolor='red',alpha=0.1)
	lines = line1 + line2 + line3 + line4
	ax2.set_xlabel('ROS Time (s)',size=35)
	ax2.set_ylabel('Number of Samples',size=35)
	ax.legend(lines, [l.get_label() for l in lines], loc=9, prop={'size': 26})

	plt.show()

	plt.rcParams.update({'font.size': 30})
	plt.rcParams['pdf.fonttype']=42
	fig, ax = plt.subplots(1)
	line1 = ax.plot(homo_t, homo_mean_average_variance,lw=2, label="homogeneous average_variance error", color = 'blue')
	ax.fill_between(homo_t, homo_mean_average_variance+homo_std_average_variance, homo_mean_average_variance-homo_std_average_variance, facecolor='blue',alpha=0.1)

	line2 = ax.plot(hetero_t, hetero_mean_average_variance,lw=2, label="heterogeneous average_variance error", color = 'red')
	ax.fill_between(hetero_t, hetero_mean_average_variance+hetero_std_average_variance, hetero_mean_average_variance-hetero_std_average_variance, facecolor='red',alpha=0.1)
	ax.set_ylabel('average_variance (uW)', size=35)
	ax.set_xlabel('ROS Time (s)',size=35)
	ax.legend(loc=0)
	ax2 = ax.twinx()
	line3 = ax2.plot(homo_t, homo_mean_num_samples,lw=2, label="homogeneous number of samples", color = 'blue',linestyle="--")
	ax2.fill_between(homo_t, homo_mean_num_samples+homo_std_num_samples, homo_mean_num_samples-homo_std_num_samples, facecolor='blue',alpha=0.1)
	# plt.axvline(x=25, color="maroon", lw=2.5)
	# plt.text(15, -33, 'Ariel Robot \nBattery Died',size=26)
	line4 = ax2.plot(hetero_t, hetero_mean_num_samples,lw=2, label="heterogeneous number of samples", color = 'red',linestyle="--")
	ax2.fill_between(hetero_t, hetero_mean_num_samples+hetero_std_num_samples, hetero_mean_num_samples-hetero_std_num_samples, facecolor='red',alpha=0.1)
	lines = line1 + line2 + line3 + line4
	ax2.set_xlabel('ROS Time (s)',size=35)
	ax2.set_ylabel('Number of Samples',size=35)
	ax.legend(lines, [l.get_label() for l in lines], loc=9, prop={'size': 26})

	plt.show()


if __name__ == '__main__':
	main()