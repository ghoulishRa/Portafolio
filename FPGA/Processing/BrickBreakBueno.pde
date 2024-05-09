import processing.serial.*;

Serial puerto;  
int pot;
Ball ball;
Paddle paddle;
Block[][] block;
int rows, columns, score, lives, level;
PImage fondo;

void setup(){
  printArray(Serial.list());
  puerto = new Serial(this, Serial.list()[3], 9600);
  fondo = loadImage("fondoJuego.png");
  fondo.resize(1950, 950);
  size(1950,950);
  textSize(24);
  level = 0;
  score=0;
  lives = 3;  
  rows = 4;
  columns = 8;
  makeLevel(columns, rows);
 
 
}

void draw(){
  while (puerto.available() > 0) {
    pot = puerto.read();
    //println(inByte);
  }
  background(fondo);
  ball.display(pot);
  ball.checkPaddle(paddle,pot);
  paddle.display(pot);
  showBlocks(); 
  showLives();
  showScore();
  checkLevel();
}

void checkLevel(){
  if(clearBlocks()){
    level++;
    ball.canMove = false;
    ball.y = paddle.y - ball.d/2;
    fill(0);
    rect(190, height/2 + 130, 400, 30);
    rect(250, height/2 + 160, 400, 30);    
    fill(255,0,255);  
    text("¡Felicidades, has terminado!" , 500, height/2+152);
    //text("Click anywhere to continue" , 260, height/2+182); 
    if(mousePressed){
      if(level%2==0){
        rows *= 2;
      } else {
        columns *= 2;
      }
      makeLevel(rows, columns);   
      ball.canMove = true;
    }
  }
}

void showScore(){
  strokeWeight(2);
  fill(255,0,255);
  text("Score: " + score, width - 140, height - 10); 
}
void showLives(){
  fill(255,0,255);  
  text("Lives: " + lives, 40, height - 10);  
  if(lives == 3){
   puerto.write('3');
  }else if(lives == 2){
   puerto.write('2');
  }else if(lives == 1){
   puerto.write('1');
  }else if(lives == 0){
   puerto.write('0');
  }
  if(lives == 0){
    fill(0);
    rect(190, height/2 + 130, 400, 30);
    rect(250, height/2 + 160, 400, 30);    
    fill(255,0,255);  
    text("¡Game Over!" , 500, height/2+250);
    //text("Click anywhere to restart" , 260, height/2+182);    
  }
}

void showBlocks(){
  for(int i = 0; i < block.length; i++){
    for(int j = 0; j < block[0].length; j++){
      block[i][j].display();
      block[i][j].checkBall(ball);
    }
  }  
}

void makeLevel(int rows, int columns){
  ball = new Ball();
  paddle = new Paddle();
  block = new Block[rows][columns];
  for(int i = 0; i < block.length; i++){
    for(int j = 0; j < block[0].length; j++){
      block[i][j] = new Block(i,j+5,block.length);
    }
  }  
}

boolean clearBlocks(){
  for(int i = 0; i < block.length; i++){
    for(int j = 0; j < block[0].length; j++){
      if(block[i][j].status){
        return false;  
      }
    }
  }
  return true;
}

void mousePressed(){
  if(lives > 0){
    ball.y -= 5;
    ball.canMove = true; 
  } else {
    setup();
  } 
  //file.stop();
  //file.play();
}
