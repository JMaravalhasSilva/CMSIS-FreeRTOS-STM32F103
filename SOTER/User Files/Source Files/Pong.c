
#include "Pong.h"
#include "LCDdriver.h"
#include <stdlib.h>
#include <stdio.h>

enum paddle_size{
	paddle_width=20,
	paddle_height=4
};

enum ball_rad{
	ball_size=5
};

typedef struct paddle_type{
	float x_pos;
	float y_pos;
	float x_pos_previous;
	float x_speed;
	uint8_t score;
}paddle_type;

typedef struct ball_type{
	float x_pos;
	float y_pos;
	float x_pos_previous;
	float y_pos_previous;
	float x_speed;
	float y_speed;
}ball_type;

//File-scope variables
static paddle_type cpu_paddle, player_paddle;
static ball_type ball;
static uint8_t difficulty=1,difficulty_counter=0,show_difficulty=0;

//File-scope function to reset ball
static void resetBall(void);


void gameInit(void){
	lcd_draw_fillrect(0,0,LCD_WIDTH,LCD_HEIGHT,BLACK);

	cpu_paddle.x_pos=LCD_WIDTH/2;
	cpu_paddle.y_pos=paddle_height/2;
	cpu_paddle.x_pos_previous=cpu_paddle.x_pos;
	cpu_paddle.x_speed=0;
	cpu_paddle.score=0;

	player_paddle.x_pos=LCD_WIDTH/2;
	player_paddle.y_pos=LCD_HEIGHT-paddle_height/2;
	player_paddle.x_pos_previous=player_paddle.x_pos;
	player_paddle.x_speed=0;
	player_paddle.score=0;

	resetBall();

	ball.x_pos_previous=ball.x_pos;
	ball.y_pos_previous=ball.y_pos;

	difficulty=1;
	difficulty_counter=0;
	show_difficulty=0;
}


void logic(float gx){

	//Limit user input
	if(gx>0.7f){
		gx=0.7f;
	}
	if(gx<-0.7f){
		gx=-0.7f;
	}

	//Change player speed according to gx or brake the pad with a 30% speed decay per frame for easier gameplay if gx is very low
	if(gx>0.1f || gx<-0.1f){
		player_paddle.x_speed-=gx*2.0f;
	}else{
		player_paddle.x_speed-=0.3f*player_paddle.x_speed;
	}

	//Add speed to CPU paddle
	if((cpu_paddle.x_pos-ball.x_pos) < 0){
		//Move paddle to the right
		cpu_paddle.x_speed +=0.1*difficulty;
	}else if((cpu_paddle.x_pos-ball.x_pos) > 0){
		//Move paddle to the left
		cpu_paddle.x_speed -=0.1*difficulty;
	}

	//Cap player paddle speed. Max speed is 10pixels/frames
	if(player_paddle.x_speed > 10){
		player_paddle.x_speed=10;
	}else if(player_paddle.x_speed < -10){
		player_paddle.x_speed=-10;
	}

	//Cap CPU paddle speed
	if(player_paddle.x_speed > 10){
		cpu_paddle.x_speed=10;
	}else if(player_paddle.x_speed < -10){
		cpu_paddle.x_speed=-10;
	}

	//Move player
	player_paddle.x_pos_previous=player_paddle.x_pos;
	player_paddle.x_pos+=player_paddle.x_speed;

	//Move CPU
	cpu_paddle.x_pos_previous=cpu_paddle.x_pos;
	cpu_paddle.x_pos+=cpu_paddle.x_speed;

	//Cap player position
	if(player_paddle.x_pos > LCD_WIDTH-(paddle_width/2)){
		player_paddle.x_pos=LCD_WIDTH-(paddle_width/2);
		player_paddle.x_speed=0;
	}else if(player_paddle.x_pos < (paddle_width/2)){
		player_paddle.x_pos=(paddle_width/2);
		player_paddle.x_speed=0;
	}

	//Cap CPU position
	if(cpu_paddle.x_pos > LCD_WIDTH-(paddle_width/2)){
		cpu_paddle.x_pos=LCD_WIDTH-(paddle_width/2);
		cpu_paddle.x_speed=0;
	}else if(cpu_paddle.x_pos < (paddle_width/2)){
		cpu_paddle.x_pos=(paddle_width/2);
		cpu_paddle.x_speed=0;
	}

	//Check ball collision with walls
	if ( (ball.x_pos >= LCD_WIDTH - (ball_size)) || (ball.x_pos <= (ball_size)) ) {
		ball.x_speed = -ball.x_speed;
	}

	//Move ball
	ball.x_pos_previous=ball.x_pos;
	ball.y_pos_previous=ball.y_pos;
	ball.x_pos += ball.x_speed;
	ball.y_pos += ball.y_speed;

	//Ball collision with CPU paddle
	if (ball.y_pos < (paddle_height*2)+(ball_size/2)) {
		//if Y position is suitable for ball collision, check X position (+-3 adds some tolerance to collision)
		if (ball.x_pos >= cpu_paddle.x_pos - (paddle_width/2) - 3 && ball.x_pos <= cpu_paddle.x_pos + ((paddle_width/2)) + 3) {
			//Invert Y speed and increase 1
			ball.y_speed = -ball.y_speed;
			if(ball.y_speed>0){
				ball.y_speed++;
			}else{
				ball.y_speed--;
			}
			//Add some of the paddle speed to the ball
			ball.x_speed += 0.2*cpu_paddle.x_speed;

			//Reset ball position so it doesn't collide with the paddle again
			ball.y_pos = (paddle_height*2)+(ball_size/2);
		}
	}

	//Ball collision with player paddle
	if (ball.y_pos > LCD_HEIGHT - (paddle_height*2)-(ball_size/2)) {

		if (ball.x_pos >= player_paddle.x_pos - (paddle_width/2) - 3 && ball.x_pos <= player_paddle.x_pos + ((paddle_width/2)) + 3) {
			ball.y_speed = -ball.y_speed;
			if(ball.y_speed>0){
				ball.y_speed++;
			}else{
				ball.y_speed--;
			}
			ball.x_speed += 0.2*player_paddle.x_speed;
			ball.y_pos = LCD_HEIGHT - (paddle_height*2)-(ball_size/2);
		}
	}

	//Cap ball x speed
	if(ball.x_speed>5.0f){
		ball.x_speed=5.0f;
	}else if(ball.x_speed<-5.0f){
		ball.x_speed=-5.0f;
	}

	//Cap ball y speed
	if(ball.y_speed>5.0f){
		ball.y_speed=5.0f;
	}else if(ball.y_speed<-5.0f){
		ball.y_speed=-5.0f;
	}

	//CPU scores goal
	if(ball.y_pos>=LCD_HEIGHT - ball_size){
		resetBall();
		cpu_paddle.score++;
	}else if(ball.y_pos<ball_size){
		resetBall();
		player_paddle.score++;
		difficulty_counter++;
		if(difficulty_counter==5){
			difficulty++;
			show_difficulty=1;
			difficulty_counter=0;
		}
	}
}



void render(TickType_t* previousWakeTime){
	static char buffer[20];

	if(player_paddle.score==50){
		//Player wins
		lcd_draw_fillrect(0,0,LCD_WIDTH,LCD_HEIGHT,BLACK);
		sprintf(buffer,"You Win!!!! :D");
		lcd_draw_string((LCD_WIDTH/2)-40,LCD_HEIGHT/2,buffer,WHITE,1);
		vTaskDelay(5000/portTICK_PERIOD_MS);
		*previousWakeTime = xTaskGetTickCount();
		gameInit();
		return;
	}else if(cpu_paddle.score==50){
		//CPU wins
		lcd_draw_fillrect(0,0,LCD_WIDTH,LCD_HEIGHT,BLACK);
		sprintf(buffer,"You Lose... :(");
		lcd_draw_string((LCD_WIDTH/2)-40,LCD_HEIGHT/2,buffer,WHITE,1);
		vTaskDelay(5000/portTICK_PERIOD_MS);
		*previousWakeTime = xTaskGetTickCount();
		gameInit();
		return;
	}

	if(show_difficulty==1){

		lcd_draw_fillrect(0,0,LCD_WIDTH,LCD_HEIGHT,BLACK);
		sprintf(buffer,"Difficulty: %u",difficulty);
		lcd_draw_string((LCD_WIDTH/2)-40,LCD_HEIGHT/2,buffer,WHITE,1);

		vTaskDelay(2000/portTICK_PERIOD_MS);
		//Refresh last tick count, otherwise this will mess the timing of the task
		*previousWakeTime = xTaskGetTickCount();
		show_difficulty=0;
		lcd_draw_fillrect(0,0,LCD_WIDTH,LCD_HEIGHT,BLACK);
	}else{
		//Delete old ball and paddles positions.
		lcd_draw_fillcircle(ball.x_pos_previous,ball.y_pos_previous,ball_size,0x0);
		lcd_draw_fillrect(cpu_paddle.x_pos_previous-(paddle_width/2),cpu_paddle.y_pos-(paddle_height/2),paddle_width,paddle_height,0x0);
		lcd_draw_fillrect(player_paddle.x_pos_previous-(paddle_width/2),player_paddle.y_pos-(paddle_height/2),paddle_width,paddle_height,0x0);
	}


	//Draw current paddle positions
	lcd_draw_fillcircle(ball.x_pos,ball.y_pos,ball_size,0xFFFF);
	lcd_draw_fillrect(cpu_paddle.x_pos-(paddle_width/2),cpu_paddle.y_pos-(paddle_height/2),paddle_width,paddle_height,0xFFFF);
	lcd_draw_fillrect(player_paddle.x_pos-(paddle_width/2),player_paddle.y_pos-(paddle_height/2),paddle_width,paddle_height,0xFFFF);
	itoa((int)cpu_paddle.score,buffer,10);
	lcd_draw_string(LCD_WIDTH-20,40,buffer,0xFFFF,1);
	itoa((int)player_paddle.score,buffer,10);
	lcd_draw_string(LCD_WIDTH-20,LCD_HEIGHT-40,buffer,0xFFFF,1);

}

static void resetBall(void){

	//Cheap and quick way to give the ball a random speed
	ball.x_speed = (-5) + (rand() % (int)(5 - (-5) + 1));
	ball.y_speed = (-2) + (rand() % (int)(2 - (-2) + 1));
	if(ball.y_speed==0){
		ball.y_speed=-1;
	}
	//If the x speed is zero, the "AI" goes crazy and fails on almost any difficulty
	if(ball.x_speed==0){
		ball.x_speed=1;
	}
	ball.x_pos=LCD_WIDTH/2;
	ball.y_pos=LCD_HEIGHT/2;
}
