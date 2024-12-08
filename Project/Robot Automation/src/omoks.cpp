#include <unistd.h>
#include<termios.h>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <vector>
#include <deque>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

#define endl '\n'

using namespace std;
#define N_GRID 15
int map_omok[N_GRID][N_GRID] ={0,};
int coordinate_human[2] = {-1,};
//Weight Board
int w_board[N_GRID][N_GRID];

int dir[8][2] = { { 1,0 },{ 0,1 },{ 1,1 },{ 1,-1 },{ -1,0 },{ 0,-1 },{ -1,-1 },{ -1,1 } };

int n = N_GRID;
int w[2][6] = { { 0,1,50,9999,500000,10000000 },{ 0,1,12,250,400000,10000000 } };

int w2[2][6][3][2];

int stx, sty;

int ansx, ansy;

ros::Publisher publisher_omok;
ros::Subscriber subscriber_omok;

struct info {
	int num , enemy, emptyspace;
};

struct info2 {
	int x, y, weight;
};

bool cmp(info2 a, info2 b) {
	return a.weight > b.weight;
}

void visualize_boardstates();
void add_weight(int color[2]);
void search(int cnt, int color);

/*This calculates next move for the AI player*/
void AI(int user_color, int ai_color);
void input(int type);

bool check_finish(int color);

void init() {
	w2[0][1][0][0] = 2; w2[1][1][0][0] = 1;
	w2[0][1][0][1] = 2; w2[1][1][0][1] = 0;
	w2[0][2][0][0] = 25, w2[1][2][0][0] = 4;
	w2[0][2][0][1] = 25, w2[1][2][0][1] = 1;
	w2[0][2][1][1] = 2; w2[1][2][1][1] = 1;
	w2[0][2][1][0] = 2; w2[1][2][1][0] = 1;
	w2[0][3][0][0] = 521, w2[1][3][0][0] = 105;
	w2[0][3][0][1] = 301; w2[1][3][0][1] = 13;
	w2[0][3][1][0] = 301, w2[1][3][1][0] = 13;
	w2[0][3][1][1] = 301, w2[1][3][1][1] = 13;
	w2[0][4][0][0] = 21000; w2[0][4][1][0] = 20010; w2[0][4][2][0] = 20010;
	w2[1][4][0][0] = 4001; w2[1][4][1][0] = 4001; w2[1][4][2][0] = 4001;
}

void add_weight(int color[2]) {
	memset(w_board, 0, sizeof(w_board));
	for (int type = 0; type < 2; type++) {
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				int sum = 0;
				info Count_omoks[5];
				if (map_omok[i][j])continue;
				for (int d = 0; d < 4; d++) {
					int nx, ny;
					int cnt = 1;
					int zerocnt = 0;
					int zerocnt1 = 0;
					int remember = 0;
					int zerocnt2 = 0;
					int num = 0;
					int enemy_cnt = 0;
					int before;

					while (true) {
						nx = i + (cnt * dir[d][0]), ny = j + (cnt * dir[d][1]);
						before = map_omok[nx - dir[d][0]][ny - dir[d][1]];
						if (nx < 0 || ny < 0 || nx >= n || ny >= n) {
							if (remember || zerocnt1 == 0) {
								enemy_cnt++;
							}
							if (before != 0)remember = zerocnt1;

							break;
						}
						if (map_omok[nx][ny] == color[(type + 1) % 2]) {
							if (remember || zerocnt1 == 0) {
								enemy_cnt++;
							}
							if (before != 0)
								remember = zerocnt1;

							break;
						}
						if (map_omok[nx][ny] == color[type]) {
							remember = zerocnt1;
							num++;
						}
						if (map_omok[nx][ny] == 0)zerocnt1++;
						if (zerocnt1 >= 2)break;
						cnt++;
					}
					zerocnt1 = remember;
					cnt = 1;
					remember = 0;
			
					while (true) {
						nx = i + (cnt * dir[d + 4][0]), ny = j + (cnt * dir[d + 4][1]);
						if (nx < 0 || ny < 0 || nx >= n || ny >= n) {
							if (remember || zerocnt2 == 0) {
								enemy_cnt++;
							}
							if (before != 0)remember = zerocnt2;
							break;
						}
						if (map_omok[nx][ny] == color[(type + 1) % 2]) {
							if (remember || zerocnt2 == 0) {
								enemy_cnt++;
							}
							if (before != 0)remember = zerocnt2;
							break;
						}

						if (map_omok[nx][ny] == color[type]) {
							remember = zerocnt2;
							num++;
						}
						if (map_omok[nx][ny] == 0)zerocnt2++;
						if (zerocnt2 >= 2)break;
						cnt++;
					}
					zerocnt2 = remember;
					zerocnt = zerocnt1 + zerocnt2;
					Count_omoks[d] = { num,enemy_cnt,zerocnt };
				}
				
				for (int d = 0; d < 4; d++) {
					int num = Count_omoks[d].num;
					int enemy = Count_omoks[d].enemy;
					int emptyspace = Count_omoks[d].emptyspace;
					int temp_w = w2[(type + 1) % 2][num][enemy][emptyspace]; 
					if (emptyspace >= 2 || num + emptyspace >= 5)continue;
					if (num != 4 && enemy >= 2)continue;
					sum += temp_w;
				}
				w_board[i][j] += sum;
				if (map_omok[i][j])w_board[i][j] = 0;
			}
		}
	}
}

bool tf;
void search(int cnt, int color)
{
	int ncolor[2] = { 0, };
	if (color == 1) {
		ncolor[0] = 2, ncolor[1] = 1;
	}
	else {
		ncolor[0] = 1, ncolor[1] = 2;
	}
	int high = 0;
	add_weight(ncolor);
	deque <info2> save_pos;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			int wow = w_board[i][j];
			if (wow) {
				if (wow == 301 || wow == 302)wow = 24;
				else if (wow >= 118 && wow <= 200)wow = 320;
				save_pos.push_back({ i,j,wow });
				high = max(high, wow);
			}
		}
	}
	sort(save_pos.begin(), save_pos.end(), cmp);

	int MAX = save_pos[0].weight;
	int idx = 0;
	for (int i = 1; i < save_pos.size(); i++) {
		idx = i;
		int num = save_pos[i].weight;
		if (num != MAX)break;
	}
	save_pos.erase(save_pos.begin() + idx, save_pos.end());

	int temp_color;
	if (color == 1)temp_color = 2;
	else temp_color = 1;
	if (cnt % 2 == 1 && check_finish(temp_color)) {
		return;
	}
	if (tf) return;

	if (!tf && (cnt % 2 == 1 && ((MAX >= 326 && MAX < 406) || MAX >= 521))) {
		if (!((105 <= MAX && MAX <= 300) || (4000 <= MAX && MAX < 20000))) {
			tf = true;
			ansx = stx, ansy = sty;
			return;
		}
	}
	if (cnt == 30) {		
		return;
	}
	if (color == 1) {
		for (int i = 0; i < save_pos.size(); i++) {
			int x = save_pos[i].x, y = save_pos[i].y;
			map_omok[x][y] = color;
			search(cnt + 1, 2);
			map_omok[x][y] = 0;
		}
	}
	else if (color == 2) {
		for (int i = 0; i < save_pos.size(); i++) {
			int x = save_pos[i].x, y = save_pos[i].y;
			map_omok[x][y] = color;
			search(cnt + 1, 1);
			map_omok[x][y] = 0;
		}
	}
}

void AI(int user_color, int ai_color) {
	tf = false;
	int color[2] = { user_color,ai_color };
	add_weight(color);
	deque <info2> save_pos;
	save_pos.clear();
	int high = 0;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			int wow = w_board[i][j];
			if (wow) {
				if (wow == 301 || wow == 302)wow = 24;
				else if (wow >= 118 && wow <= 200)wow = 320;
				save_pos.push_back({ i,j,wow });
				if (high < wow) {
					high = wow;
					ansx = i, ansy = j;
				}
			}
		}
	}

	sort(save_pos.begin(), save_pos.end(), cmp);
	int MAX = save_pos[0].weight;
	if (!((MAX >= 326 && MAX < 406) || MAX >= 521)) {
		for (int i = 0; i < save_pos.size(); i++) {
			int x = save_pos[i].x, y = save_pos[i].y;
			stx = x, sty = y;
			map_omok[x][y] = ai_color;
			search(0, user_color);
			map_omok[x][y] = 0;
		}
	}
	map_omok[ansx][ansy] = ai_color;
	
	/* Publish Message goes here */
	std_msgs::Int32MultiArray msg;
	msg.data.push_back(ansx);
	msg.data.push_back(ansy);
	ROS_INFO("Published move: %d %d",msg.data[0],msg.data[1]);
	publisher_omok.publish(msg);
	ros::spin();
}

void visualize_boardstates() {
	cout << "x|y";
	for (int j = 0; j < n; j++) {
		cout.width(3);
		cout << j;
	}
	cout << endl;
	for (int i = 0; i < n; i++) {
		cout.width(3);
		cout << i;
		for (int j = 0; j < n; j++) {
			cout.width(3);
			if (map_omok[i][j])
				cout << map_omok[i][j];
			else cout << "";
		}
		cout << endl;
	}
}

void input(int type) {
	int x, y;
	while (true) {
		
		x = coordinate_human[0];
		y = coordinate_human[1];
		if (x >= 0 && y >= 0 && x < n && y < n && map_omok[x][y] == 0) {
			cout << "Input to the algorithm"<<endl;
			map_omok[x][y] = type;
			break;
		}
	}
	cout << endl;
}

bool check_finish(int color) {
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			if (map_omok[i][j] == color) {
				for (int d = 0; d < 8; d++) {
					int cnt = 1;
					while (true) {
						int nx = i + (cnt * dir[d][0]), ny = j + (cnt * dir[d][1]);
						if (nx < 0 || ny < 0 || nx >= n || ny >= n)break;
						if (map_omok[nx][ny] != color)break;
						cnt++;
					}
					if (cnt == 5)return true;
				}
			}
		}
	}
	return false;
}


void callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
	if (msg->data.size() == 2){
		coordinate_human[0] = msg->data[0];  // First coordinate (x)
		coordinate_human[1] = msg->data[1];  // Second coordinate (y)
		ROS_INFO("Received coordinates: x=%d, y=%d", coordinate_human[0],coordinate_human[1]);
	}
	else
		ROS_WARN("Received message with unexpected data size: %lu", msg->data.size());
  return;
}

int main(int argc, char** argv) {
	init();
	ros::init(argc, argv,"int_array_publisher");
	ros::NodeHandle node_handler1;
	ros::NodeHandle node_handler2;
	publisher_omok = node_handler1.advertise<std_msgs::Int32MultiArray>("int_array_topic",2);
	subscriber_omok= node_handler2.subscribe<std_msgs::Int32MultiArray>("newnew_stone",10,callback);
	ros::Rate loop_rate(1);

	cout << "\n well come, Five dols!" << endl;
	cout << "your color is black(1). please input 1! " << endl << endl;
	int turn = 0;
	while (true) {
		cout << "Your turn" << endl;
		visualize_boardstates();
		input(1);

		tf = check_finish(1);
		if (tf) {
			visualize_boardstates();
			std_msgs::Int32MultiArray msg;
			msg.data.push_back(-100);
			msg.data.push_back(-100);
			ROS_INFO("Published move: %d %d",msg.data[0],msg.data[1]);
			publisher_omok.publish(msg);
			cout << " you win!" << endl;
			return 0;
		}

		//AI turn
		cout << "AI turn" << endl;
		visualize_boardstates();
		usleep(1000);
		AI(1, 2);
		tf = check_finish(2);
		if (tf) {
			visualize_boardstates();
			std_msgs::Int32MultiArray msg;
			msg.data.push_back(-100);
			msg.data.push_back(-100);
			ROS_INFO("Published move: %d %d",msg.data[0],msg.data[1]);
			publisher_omok.publish(msg);
			cout << " AI win!" << endl;
			
			return 0;
		}
		turn++;
		if(getchar()) break;
	}
}


		