#include "json_parse.h"

using namespace nubot;

JSONparse::JSONparse()
{
    chafinal_=' ';
}

JSONparse::~JSONparse()
{

}

bool JSONparse::parseJSON_(const QByteArray &datagram)
{
    chafinal_=' ';
    QJsonParseError json_error;
    QJsonDocument parse_doucment = QJsonDocument::fromJson(datagram, &json_error);
    if(json_error.error == QJsonParseError::NoError)
    {
        if(parse_doucment.isObject())
        {
            QJsonObject obj = parse_doucment.object();
            if(obj.contains("eventCode"))
            {
                QJsonValue control_value = obj.take("eventCode");
                if(control_value.isString())
                {
                    char *tmp = control_value.toString().toLatin1().data();
                    chafinal_=*tmp;
                    //qDebug()<<chafinal_;
                }
            }
            if(obj.contains("scoreTeamA"))
            {
                QJsonValue scoreA_value = obj.take("scoreTeamA");
                if(scoreA_value.isDouble())
                {
                    score_cyan_ = scoreA_value.toInt();
                }
            }
            if(obj.contains("scoreTeamB"))
            {
                QJsonValue scoreB_value = obj.take("scoreTeamB");
                if(scoreB_value.isDouble())
                {
                    score_magenta_ = scoreB_value.toInt();
                }
            }
            return true;
        }
    }
}
