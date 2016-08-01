#ifndef JSONPARSE_H
#define JSONPARSE_H

#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <QFile>
#include <QDebug>


#define COMM_STOP                'S'
#define COMM_START               's'
#define COMM_HALT                'H'
#define COMM_READY               ' '
#define COMM_PARKING             'L'
#define COMM_FIRST_HALF          '1'
#define COMM_HALF_TIME           'h'
#define COMM_SECOND_HALF         '2'
#define COMM_END_GAME            'e'
#define COMM_CANCEL              'x'
#define COMM_GOAL_MAGENTA        'a'
#define COMM_GOAL_CYAN           'A'
#define COMM_SUBGOAL_MAGENTA     'd'
#define COMM_SUBGOAL_CYAN        'D'
#define COMM_RESTART             'n'
#define COMM_KICKOFF_MAGENTA     'k'
#define COMM_KICKOFF_CYAN        'K'
#define COMM_FREEKICK_MAGENTA    'f'
#define COMM_FREEKICK_CYAN       'F'
#define COMM_GOALKICK_MAGENTA    'g'
#define COMM_GOALKICK_CYAN       'G'
#define COMM_THROWIN_MAGENTA     't'
#define COMM_THROWIN_CYAN        'T'
#define COMM_CORNER_MAGENTA      'c'
#define COMM_CORNER_CYAN         'C'
#define COMM_PENALTY_MAGENTA     'p'
#define COMM_PENALTY_CYAN        'P'
#define COMM_DROPPED_BALL        'N'

using namespace std;
namespace nubot {

class JSONparse
{
public:
    char chafinal_;                  //裁判盒的控制信息
    int score_cyan_;
    int score_magenta_;              //当前的比分信息

public:
    JSONparse();
    ~JSONparse();

    bool parseJSON_(const QByteArray & datagram);     //解析JSON文件
};
}
#endif // JSONPARSE_H
