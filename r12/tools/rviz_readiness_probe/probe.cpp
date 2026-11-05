#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QTimer>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QSaveFile>
#include <chrono>
namespace r12 {
class ReadinessProbe : public rviz_common::Panel {
public:
 ReadinessProbe():Panel(nullptr) {}
 void onInitialize() override {
  auto timer=new QTimer(this);
  connect(timer,&QTimer::timeout,this,[this](){sample();});
  timer->start(250);
 }
private:
 void collect(rviz_common::properties::Property *p,QJsonArray &rows,bool &ready) {
  if(auto s=dynamic_cast<rviz_common::properties::StatusProperty*>(p)) {
   rows.append(QJsonObject{{"name",s->getName()},{"text",s->getValue().toString()},{"level",int(s->getLevel())}});
   if(s->getName()=="URDF" && s->getLevel()==rviz_common::properties::StatusProperty::Ok && s->getValue().toString()=="URDF parsed OK")ready=true;
  }
  for(int i=0;i<p->numChildren();++i)collect(p->childAt(i),rows,ready);
 }
 void sample() {
  const auto path=qEnvironmentVariable("R12_RVIZ_READINESS_FILE");if(path.isEmpty())return;
  auto group=getDisplayContext()->getRootDisplayGroup();QJsonArray models;bool all_ready=true;int count=0;
  for(int i=0;i<group->numDisplays();++i){auto d=group->getDisplayAt(i);
   if(d->getClassId()!="rviz_default_plugins/RobotModel" || !d->isEnabled())continue;
   ++count;bool ready=false;QJsonArray statuses;collect(d,statuses,ready);all_ready=all_ready&&ready;
   models.append(QJsonObject{{"name",d->getName()},{"statuses",statuses},{"urdf_ready",ready}});
  }
  auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  QJsonObject row{{"monotonic_ns",QString::number(ns)},{"models",models},{"urdf_ready",count>0&&all_ready},{"scope","URDF parsing status only; not TF, sensor, or mission readiness"}};
  QSaveFile file(path);if(file.open(QIODevice::WriteOnly)){file.write(QJsonDocument(row).toJson());file.commit();}
 }
};
}
PLUGINLIB_EXPORT_CLASS(r12::ReadinessProbe,rviz_common::Panel)
