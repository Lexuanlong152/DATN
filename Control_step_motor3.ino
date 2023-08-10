#include <AccelStepper.h>

/* Khai báo các chân sử dụng trên board */
#define limitswitch 3   /*Chân tín hiệu của công tắc tiệm cận*/
#define pin_PUL 2   /* Chân điều khiển xung cho động cơ */
#define pin_DIR 5   /* Chân điều khiển chiều xoay của động cơ */

/* Các trạng thái trong State Machine để điều khiển động cơ*/
#define IDLE 1
#define HOME 2
#define RE_RUN  3
#define RUNNING  4
#define STOP 5
#define TARGET 6
#define RIGHT_TARGET 7
#define LEFT_TARGET 8
#define MID_TARGET 9
#define SEEK_LEFT 10
#define SEEK_RIGHT 11

/* Các vị trí biên & vị trí cố định*/
#define LEFT_PUL  65500
#define RIGHT_PUL 500
#define MID_PUL 33000

int ratio = 16;    /* Tỉ lệ chia xung trên Driver*/
float pul_circle = 360.0/(1.8/ratio); /* Số xung để xoay được 1 vòng */
unsigned long positionStep = 0 ; /*Vị trí của động cơ*/
int R = 1200;   /* Bán kính của bánh răng của động cơ */
int state_motor = IDLE ; /* Biến lưu trạng thái của động cơ */
String g_data = ""; /* Buffer chứa lệnh */
bool flag_new_data = false; /*Báo hiệu nhận được lệnh mới từ Jetson*/
bool flag_new_state = true;  /* Báo hiệu chuyển trạng thái */ 
bool flag_home = false; /* Báo hiệu về vị trí home */
bool flag_target = false;   /* Báo hiệu về vị trí đích */
int dir_target = 0;   /* Chiều xoay tại vị trí đích */

unsigned long pos_cur = 0;  /* Vị trí hiện tại của động cơ */
unsigned long pos_next = 0;  /* Vị trí tiếp theo của động cơ*/
unsigned long soXung_int = 0 ; /* Số xung cần thiết */
int dir_motor = -1; /* Chiều xoay của động cơ | 1 : trái sang phải | -1 : phải sang trái */

/* Khởi tạo class điều khiển động cơ Step */
AccelStepper stepper1(1,pin_PUL,pin_DIR); // 1: chế độ dùng driver | 2: chân step | 5: Chân dir

/*
Tên hàm: setup() 
Chức năng : Khởi tạo các giá trị duy nhất một lần sau khi khởi động
*/ 
void setup() {
  /* Khởi tạo port giao tiếp UART với baudrate = 9600 */
  Serial.begin(9600);
  /* Thiết lập tốc độ tối đa cho động cơ Step (số xung / giây )*/
  /* Thiết lập tốc độ tối đa của động cơ*/
  stepper1.setMaxSpeed(4000);
  /* Thiết lập gia tốc cho động cơ Step */
  /* Thiết lập gia tốc của động cơ*/
  stepper1.setAcceleration(4000);
  /* Thiết lập vị trí hiện tại là gốc toạ độ của trục */
  stepper1.setCurrentPosition(0);
  /* Khởi tạo chân Input để đọc tín hiệu của công tắc tiệm cận*/
  pinMode(limitswitch, INPUT_PULLUP);
  /* Thiết lập chế độ Interrupt bằng phần cứng cho chân Input*/
  attachInterrupt(digitalPinToInterrupt(limitswitch), readSensor, CHANGE);
  
}

/*
Tên hàm: readSensor() 
Chức năng : Hàm bật cờ tín hiệu về vị trí gốc (Home) khi Interrupt xảy ra
*/ 
void readSensor()
{
  /* Nếu chưa được thiết lập về vị trí Home , thì bật cờ báo hiệu*/
  /* Báo hiệu về vị trí home */
  if(flag_home == false)/* Báo hiệu về vị trí home */
  {
    /* Báo hiệu về vị trí home */
   flag_home = true;/* Báo hiệu về vị trí home */
  }  
}

/*
Tên hàm: control_motor() 
Chức năng : Hàm điều khiển động cơ Step bằng State machine
*/ 
void control_motor(String data){
   long est_pos = 212;  /*Biến để vị trí dự kiến sẽ dừng lại*/
   switch (state_motor)
   {
    /* Trạng thái chờ của động cơ, động cơ đứng yên chờ chuyển sang trạng thái khác*/
    case IDLE:    
     /* Nếu có sự thay đổi trạng thái */
     if (flag_new_state == true){
          /* Khởi tạo lại cờ tín hiệu*/
          flag_new_state = false;
          /* Thiết lập tốc độ tối đa của động cơ*/
          stepper1.setMaxSpeed(4000);
          /* Thiết lập tốc độ hiện tại của động cơ*/
          stepper1.setSpeed(4000);
          /* Thiết lập gia tốc của động cơ*/
          stepper1.setAcceleration(2000);
          /* Khởi tạo lại cờ tín hiệu*/
          /* Báo hiệu về vị trí home */
          flag_home = false;   /* Báo hiệu về vị trí home */
          flag_target = false; /* Báo hiệu trạng thái trước đó là vị trí đích*/
          dir_target = 0;      /* Chiều của động cơ tại vị trí đích*/
          /* Gửi phản hồi về Jetson */
          Serial.println("IDLE");
     }
    
    /* Nhận lệnh "Về Home" từ Jetson*/
    if (data.indexOf("HOME") >= 0)
    {      
      g_data = ""; /* Xoá buffer */   
      flag_new_state = true;   /* Báo hiệu chuyển trạng thái */ 
      /* Chuyển trạng thái sang HOME */
      state_motor = HOME ;
    }

    /* Nhận lệnh "Quay động cơ tới vị trí bất kì" từ Jetson*/
    if (data.indexOf("RUN") >= 0)
    {      
      g_data = ""; /* Xoá buffer */  
      flag_new_state = true;   /* Báo hiệu chuyển trạng thái */     
      /* Chuyển trạng thái sang RERUN : chuẩn bị và chờ vị trí cần tới */
      state_motor = RE_RUN ;
    }

    /* Nhận lệnh "Tìm kiểm vật thể bên trái" từ Jetson*/
    if (data.indexOf("SL") >= 0)
    {
      g_data = ""; /* Xoá buffer */  
      flag_new_state = true;   /* Báo hiệu chuyển trạng thái */ 
      /* Chuyển trạng thái sang SEEK_LEFT */
      state_motor = SEEK_LEFT ;
    }

    /* Nhận lệnh "Tìm kiểm vật thể bên phải" từ Jetson*/
    if (data.indexOf("SR") >= 0)
    {
      g_data = ""; /* Xoá buffer */      
      flag_new_state = true;   /* Báo hiệu chuyển trạng thái */ 
      /* Chuyển trạng thái sang SEEK_RIGHT */
      state_motor = SEEK_RIGHT ;
    }

    /* Nhận lệnh "Di chuyển tới vị trí giữa" từ Jetson*/
    if (data.indexOf("MID") >= 0)
    {
      g_data = ""; /* Xoá buffer */      
      flag_new_state = true;   /* Báo hiệu chuyển trạng thái */ 
      /* Chuyển trạng thái sang MID_TARGET */
      state_motor = MID_TARGET ;
    }
 
    /* Nhận lệnh "Di chuyển tới vị trí đích bên Trái" từ Jetson*/
    if (data.indexOf("LEFT") >= 0)
    {
      g_data = ""; /* Xoá buffer */     
      flag_new_state = true;   /* Báo hiệu chuyển trạng thái */ 
      flag_target = true;  /* Báo hiệu về vị trí đích */ 
      /* Chuyển trạng thái sang LEFT_TARGET */
      state_motor = LEFT_TARGET ;
    }

    /* Nhận lệnh "Di chuyển tới vị trí đích bên Phải" từ Jetson*/
    if (data.indexOf("RIGHT") >= 0)
    {
      g_data = ""; /* Xoá buffer */     
      flag_new_state = true;   /* Báo hiệu chuyển trạng thái */ 
      flag_target = true;    /* Báo hiệu về vị trí đích */ 
      /* Chuyển trạng thái sang RIGHT_TARGET */
      state_motor = RIGHT_TARGET ;      
    }  
    break;
    
    /* Điều khiển động cơ về vị trí Home*/
    case HOME:
         /* Nếu có sự thay đổi trạng thái */
         if (flag_new_state == true){
             /* Khởi tạo lại cờ tín hiệu*/
             flag_new_state = false;
             /* Báo hiệu về vị trí home */
             flag_home = false;
         }
         /* Báo hiệu về vị trí home */
         if(flag_home == false)
        {                       
            /* Thiết lập tốc độ hiện tại của động cơ*/
            stepper1.setSpeed(-2500);
            /* Xoay động cơ với tốc độ hiện tại*/
            stepper1.runSpeed();
            /* Thiết lập vị trí hiện tại là gốc toạ độ*/
            stepper1.setCurrentPosition(0);
         }
         else{
            /* Thiết lập vị trí hiện tại là gốc toạ độ*/
            stepper1.setCurrentPosition(0);
            /* Xoay động cơ đến vị trí theo số xung yêu cầu*/
            stepper1.moveTo(212);
            /* Nếu chưa đến vị trí cần đến */
            while(stepper1.currentPosition()!=stepper1.targetPosition())
             { 
               /* Xoay động cơ bằng 1 xung */
               stepper1.run();    
             }  
            flag_new_state = true;   /* Báo hiệu chuyển trạng thái */ 
            state_motor = STOP ;  /*Chuyển trạng thái sang STOP */
         }
        break;
        
    /* Khởi tạo các thông số trước khi bắt đầu xoay động cơ và chờ vị trí cần đến */    
    case RE_RUN:
        /* Nếu có sự thay đổi trạng thái */
        if (flag_new_state == true){
             /* Khởi tạo lại cờ tín hiệu*/
             flag_new_state = false;
             /*Báo hiệu nhận được lệnh mới từ Jetson*/
             flag_new_data = false;
         }
         /*Báo hiệu nhận được lệnh mới từ Jetson*/
        if (flag_new_data == true)
        {
          /*Báo hiệu nhận được lệnh mới từ Jetson*/
          flag_new_data = false;
          /*Vị trí cần đến */
          pos_next = atol(data.c_str());
          /* Số xung cần xoay để đến vị trí cần đến*/
          soXung_int = (pul_circle*pos_next)/(2*PI*R);  
          /* Xoá buffer */        
          g_data = "";
          /* Chiều xoay của động cơ */
          dir_motor = (soXung_int >= pos_cur ) ? 1 : -1;
          /* Thiết lập tốc độ tối đa của động cơ*/
          stepper1.setMaxSpeed(1500);
          /* Thiết lập tốc độ hiện tại của động cơ*/
          stepper1.setSpeed(1500);
          /* Xoay động cơ đến vị trí theo số xung yêu cầu*/
          stepper1.moveTo(soXung_int);
          /* Chuyển trạng thái sang RUNNING*/
          state_motor = RUNNING ;
        }                
        break;    
    
    /* Xoay động cơ đến vị trí cần đến*/
    case RUNNING:    
        /* Nếu chưa đến vị trí cần đến */
        if(stepper1.currentPosition()!= stepper1.targetPosition()  && (flag_home == false))
        {          
          /* Xoay động cơ theo từng xung */
          stepper1.run();    
        }  
        else{
          /* Báo hiệu về vị trí home */
          if((flag_home == true))
          {
            /* Theo chiều từ trái sang phải */
            if ( dir_motor > 0)
            {
              /* Xoay động cơ đến vị trí theo số xung yêu cầu*/
              stepper1.moveTo(27799);
              /* Đảo chiều di chuyển*/
              dir_motor = -1 ;
            }
            else
            {
              /* Đảo chiều di chuyển*/
              dir_motor = 1 ;
              /* Xoay động cơ đến vị trí theo số xung yêu cầu*/
              stepper1.moveTo(212);
            }
            /* Nếu chưa đến vị trí cần đến */
            while(stepper1.currentPosition()!=stepper1.targetPosition())
            {
             /* Xoay động cơ theo từng xung */
              stepper1.run();    
            }  
          }
          flag_new_state = true;   /* Báo hiệu chuyển trạng thái */ 
          /* Chuyển trạng thái sang STOP */
          state_motor = STOP ;   
        }
        /* Cập nhật vị trí hiện tại của động cơ*/
        pos_cur = stepper1.currentPosition();
        
        /* Nếu có tín hiệu dừng đột ngột*/
        if ( (flag_target == false) && (Serial.available() > 0) ){
          Serial.read();
          /* Dự kiến vị trí sẽ dừng lại */
          est_pos = pos_cur + 2500*dir_motor ;
          /* Nếu vị trí lớn hơn vị trí biên Trái thì về vị trí biên */
          if (est_pos > 27798){
            est_pos = 27799;
          }
          /* Nếu vị trí lớn hơn vị trí biên Phải thì về vị trí biên */
          if (est_pos < 213){
            est_pos = 212;
          }          
          /* Xoay động cơ đến vị trí theo số xung yêu cầu*/
          stepper1.moveTo(est_pos);
        }
                
        break;

    /* Động cơ đã dừng hẳn */
    case STOP:
        /* Gửi lệnh phản hồi đã dừng động cơ và vị trí hiện tại của động cơ */
        Serial.println("STOP:" + String(stepper1.currentPosition()));    
        /* Báo hiệu chuyển trạng thái */  
        flag_new_state = true;   
        /* Chuyển trạng thái về IDLE */
        state_motor = IDLE ;
        /* Gửi lệnh phản hồi đã dừng động cơ tại vị trí đích */
        if (dir_target > 0)
        {
          /* Vị trí biên bên Trái - Target Left*/
          Serial.println("TL"); 
        }
        else if (dir_target < 0)
        {
          /* Vị trí biên bên Phải - Target Right*/
          Serial.println("TR"); 
        }
        break;
        
    /* Di chuyển về vị trí đích bên Trái */
    case LEFT_TARGET:  
        /* Thiết lập tốc độ tối đa của động cơ*/
        stepper1.setMaxSpeed(5000);
        /* Thiết lập tốc độ hiện tại của động cơ*/
        stepper1.setSpeed(4000);
        soXung_int = (pul_circle*LEFT_PUL)/(2*PI*R);
        /* Xoay động cơ đến vị trí theo số xung yêu cầu*/
        stepper1.moveTo(soXung_int);
        /* Báo hiệu về vị trí home */
        flag_home = false;
        /* Chuyển trạng thái sang RUNNING */
        state_motor = RUNNING ;
        /* Chiều di chuyển từ trái sang phải */
        dir_target = 1 ;
        /* Thiết lập gia tốc của động cơ*/
        stepper1.setAcceleration(4000);
        break
        
    /* Di chuyển về vị trí đích bên Phải */
    case RIGHT_TARGET:  
        /* Thiết lập tốc độ tối đa của động cơ*/
        stepper1.setMaxSpeed(5000);
        /* Thiết lập tốc độ hiện tại của động cơ*/
        stepper1.setSpeed(4000);
        soXung_int = (pul_circle*RIGHT_PUL)/(2*PI*R);
        /* Xoay động cơ đến vị trí theo số xung yêu cầu*/
        stepper1.moveTo(soXung_int);
        /* Báo hiệu về vị trí home */
        flag_home = false;
        /* Chuyển trạng thái sang RUNNING */
        state_motor = RUNNING ;
        /* Chiều di chuyển từ phải sang trái */
        dir_target = -1 ;
        /* Thiết lập gia tốc của động cơ*/
        stepper1.setAcceleration(4000);
        break;
    
    /* Di chuyển về vị trí giữa ray trượt */
    case MID_TARGET:  
        /* Thiết lập tốc độ tối đa của động cơ*/
        stepper1.setMaxSpeed(5000);
        /* Thiết lập tốc độ hiện tại của động cơ*/
        stepper1.setSpeed(4000);
        soXung_int = (pul_circle*MID_PUL)/(2*PI*R);
        /* Xoay động cơ đến vị trí theo số xung yêu cầu*/
        stepper1.moveTo(soXung_int);
        /* Báo hiệu về vị trí home */
        flag_home = false;
        /* Chuyển trạng thái sang RUNNING */
        state_motor = RUNNING ;
        /* Thiết lập gia tốc của động cơ*/
        stepper1.setAcceleration(4000);
        break;
    
    /* Di chuyển về phía bên trái để tìm vật thể */
    case SEEK_LEFT:  
        /* Nếu có sự thay đổi trạng thái */
        if (flag_new_state == true){
            /* Khởi tạo lại cờ tín hiệu*/
            flag_new_state = false;
            /* Vị trí cần đến */
            pos_next = 65500;
            /* Số xung cần xoay động cơ */
            soXung_int = (pul_circle*pos_next)/(2*PI*R);
            g_data = ""; /* Xoá buffer */
            /* Chiều di chuyển từ trái sang phải */
            dir_motor = 1;
            /* Thiết lập tốc độ tối đa của động cơ*/
            stepper1.setMaxSpeed(2500);
            /* Thiết lập tốc độ hiện tại của động cơ*/
            stepper1.setSpeed(2500);
            /* Xoay động cơ đến vị trí theo số xung yêu cầu*/
            stepper1.moveTo(soXung_int);
            /* Chuyển trạng thái sang RUNNING*/
            state_motor = RUNNING ;
        }         
        break;
    
    /* Di chuyển về phía bên phải để tìm vật thể */
    case SEEK_RIGHT:  
        /* Nếu có sự thay đổi trạng thái */
        if (flag_new_state == true){
            /* Khởi tạo lại cờ tín hiệu*/
            flag_new_state = false;
            /* Vị trí cần đến */
            pos_next = 500;
            /* Số xung cần xoay động cơ */
            soXung_int = (pul_circle*pos_next)/(2*PI*R);
             /* Xoá buffer */
            g_data = ""; 
            /* Chiều di chuyển từ phải sang trái */
            dir_motor = -1;
            /* Thiết lập tốc độ tối đa của động cơ*/
            stepper1.setMaxSpeed(2500);
            /* Thiết lập tốc độ hiện tại của động cơ*/
            stepper1.setSpeed(2500);
            /* Xoay động cơ đến vị trí theo số xung yêu cầu*/
            stepper1.moveTo(soXung_int);
            /* Chuyển trạng thái sang RUNNING*/
            state_motor = RUNNING ;
        }   
        break;
    default:
        flag_new_state = true;   /* Báo hiệu chuyển trạng thái */ 
        /* Chuyển trạng thái về IDLE */
        state_motor = IDLE; 
        break;
   }  
}

void loop() {
    /* Nếu động cơ đang xoay thì không nhận tín hiệu bên ngoài*/
    if(state_motor != RUNNING)
    {
      /*Khi có lệnh được gửi từ Jetson*/
      if (Serial.available() > 0) {
          /*Đọc dữ liệu nhận được từ Master */
          String data = Serial.readString(); 
          /* Lưu dữ liệu vào biến global */
          g_data = data;
          /*Báo hiệu nhận được lệnh mới từ Jetson*/
          flag_new_data = true;
        } 
    }    

    /* Điều khiển động cơ Step bằng state machine*/
    control_motor(g_data);
}
