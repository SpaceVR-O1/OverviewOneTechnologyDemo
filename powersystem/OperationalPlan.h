#ifndef OPERATIONAL_PLAN_H
#define OPERATIONAL_PLAN_H

#define DEBUG 1
#define HALF_ORBIT 45
#define FILL_ORBIT 90
#define FINAL_ORBIT 8 //2912 = 16 orbits a day for 182 days (~6 months)

class OperationalPlan
{ 
  public:
    OperationalPlan();
    OperationalPlan(bool);
    ~OperationalPlan();
    OperationalPlan(bool []);
    bool getPowerPlan(int);
    void setPowerPlan(int, bool);
        
  private:
    bool powerPlan[FINAL_ORBIT*2];

};

#endif
