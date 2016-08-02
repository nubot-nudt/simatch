/* A Bison parser, made by GNU Bison 2.5.  */

/* Bison implementation for Yacc-like parsers in C
   
      Copyright (C) 1984, 1989-1990, 2000-2011 Free Software Foundation, Inc.
   
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.
   
   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "2.5"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 1

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1

/* Using locations.  */
#define YYLSP_NEEDED 0



/* Copy the first part of user declarations.  */

/* Line 268 of yacc.c  */
#line 21 "xrtdb.y"

#include <stdio.h>
#include <stdlib.h>
#include "rtdb_configuration.h"
#include "rtdb_errors.h"
#include "rtdb_structs.h"
#include "rtdb_functions.h"
#include "rtdb_user_creator.h"
#include "rtdb_ini_creator.h"
#define YYSTYPE char*
#define YYERROR_VERBOSE 1

//Lexical analizer function prototype
int yylex(char **);
//Bison Error handling function prototype
void yyerror(char *);

//Global var to store the current input file line number for error handling
unsigned nline= 1;

//Aux global vars to create the global lists defined in rtdb_functions.h
rtdb_Item * pItem;
rtdb_Schema * pSchema;
rtdb_Assignment * pAssign;


/* Line 268 of yacc.c  */
#line 98 "xrtdb.tab.c"

/* Enabling traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

/* Enabling the token table.  */
#ifndef YYTOKEN_TABLE
# define YYTOKEN_TABLE 0
#endif


/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     agentsDECL = 258,
     itemDECL = 259,
     schemaDECL = 260,
     assignmentDECL = 261,
     datatypeFIELD = 262,
     periodFIELD = 263,
     headerfileFIELD = 264,
     sharedFIELD = 265,
     localFIELD = 266,
     schemaFIELD = 267,
     agentsFIELD = 268,
     identifier = 269,
     headerfl = 270,
     integer = 271,
     equal = 272,
     semicomma = 273,
     comma = 274,
     openbrace = 275,
     closebrace = 276,
     eol = 277,
     eof = 278
   };
#endif



#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef int YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
#endif


/* Copy the second part of user declarations.  */


/* Line 343 of yacc.c  */
#line 163 "xrtdb.tab.c"

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#elif (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
typedef signed char yytype_int8;
#else
typedef short int yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(e) ((void) (e))
#else
# define YYUSE(e) /* empty */
#endif

/* Identity function, used to suppress warnings about constant conditions.  */
#ifndef lint
# define YYID(n) (n)
#else
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static int
YYID (int yyi)
#else
static int
YYID (yyi)
    int yyi;
#endif
{
  return yyi;
}
#endif

#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined EXIT_SUCCESS && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#     ifndef EXIT_SUCCESS
#      define EXIT_SUCCESS 0
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (YYID (0))
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined EXIT_SUCCESS \
       && ! ((defined YYMALLOC || defined malloc) \
	     && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef EXIT_SUCCESS
#    define EXIT_SUCCESS 0
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined EXIT_SUCCESS && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined EXIT_SUCCESS && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
	 || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss_alloc;
  YYSTYPE yyvs_alloc;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

# define YYCOPY_NEEDED 1

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack_alloc, Stack)				\
    do									\
      {									\
	YYSIZE_T yynewbytes;						\
	YYCOPY (&yyptr->Stack_alloc, Stack, yysize);			\
	Stack = &yyptr->Stack_alloc;					\
	yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
	yyptr += yynewbytes / sizeof (*yyptr);				\
      }									\
    while (YYID (0))

#endif

#if defined YYCOPY_NEEDED && YYCOPY_NEEDED
/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
      do					\
	{					\
	  YYSIZE_T yyi;				\
	  for (yyi = 0; yyi < (Count); yyi++)	\
	    (To)[yyi] = (From)[yyi];		\
	}					\
      while (YYID (0))
#  endif
# endif
#endif /* !YYCOPY_NEEDED */

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  18
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   120

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  24
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  33
/* YYNRULES -- Number of rules.  */
#define YYNRULES  75
/* YYNRULES -- Number of states.  */
#define YYNSTATES  133

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   278

#define YYTRANSLATE(YYX)						\
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[YYLEX] -- Bison symbol number corresponding to YYLEX.  */
static const yytype_uint8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
   YYRHS.  */
static const yytype_uint8 yyprhs[] =
{
       0,     0,     3,     5,     6,    10,    11,    18,    19,    27,
      28,    34,    35,    41,    45,    47,    49,    51,    55,    57,
      58,    62,    65,    67,    68,    72,    73,    79,    80,    86,
      87,    93,    95,    97,    98,   102,   105,   107,   109,   110,
     114,   117,   119,   120,   124,   130,   135,   141,   146,   148,
     150,   152,   157,   159,   160,   162,   164,   169,   171,   172,
     176,   177,   181,   183,   184,   188,   189,   196,   197,   203,
     209,   214,   216,   218,   220,   224
};

/* YYRHS -- A `-1'-separated list of the rules' RHS.  */
static const yytype_int8 yyrhs[] =
{
      25,     0,    -1,    26,    -1,    -1,    22,    27,    26,    -1,
      -1,     3,    17,    32,    22,    28,    26,    -1,    -1,     3,
      17,    32,    18,    22,    29,    26,    -1,    -1,     4,    14,
      30,    33,    26,    -1,    -1,     5,    14,    31,    42,    26,
      -1,     6,    49,    26,    -1,    23,    -1,     1,    -1,    14,
      -1,    32,    19,    14,    -1,     1,    -1,    -1,    22,    34,
      33,    -1,    20,    35,    -1,     1,    -1,    -1,    22,    36,
      35,    -1,    -1,     7,    17,    14,    37,    40,    -1,    -1,
       8,    17,    14,    38,    40,    -1,    -1,     9,    17,    15,
      39,    40,    -1,    21,    -1,     1,    -1,    -1,    22,    41,
      35,    -1,    18,    35,    -1,    21,    -1,     1,    -1,    -1,
      22,    43,    42,    -1,    20,    44,    -1,     1,    -1,    -1,
      22,    45,    44,    -1,    10,    17,    46,    18,    44,    -1,
      10,    17,    46,    44,    -1,    11,    17,    48,    18,    44,
      -1,    11,    17,    48,    44,    -1,    21,    -1,     1,    -1,
      14,    -1,    46,    19,    47,    14,    -1,     1,    -1,    -1,
      22,    -1,    14,    -1,    48,    19,    47,    14,    -1,     1,
      -1,    -1,    22,    50,    49,    -1,    -1,    20,    51,    52,
      -1,     1,    -1,    -1,    22,    53,    52,    -1,    -1,    12,
      17,    14,    54,    18,    52,    -1,    -1,    12,    17,    14,
      55,    52,    -1,    13,    17,    56,    18,    52,    -1,    13,
      17,    56,    52,    -1,    21,    -1,     1,    -1,    14,    -1,
      56,    19,    14,    -1,     1,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const yytype_uint8 yyrline[] =
{
       0,    48,    48,    50,    50,    51,    51,    52,    52,    53,
      53,    54,    54,    55,    56,    57,    60,    61,    62,    65,
      65,    66,    67,    70,    70,    71,    71,    72,    72,    73,
      73,    74,    75,    78,    78,    79,    80,    81,    84,    84,
      85,    86,    89,    89,    90,    91,    92,    93,    94,    95,
      98,    99,   100,   103,   104,   107,   108,   109,   112,   112,
     113,   113,   114,   117,   117,   118,   118,   119,   119,   120,
     121,   122,   123,   126,   127,   128
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "agentsDECL", "itemDECL", "schemaDECL",
  "assignmentDECL", "datatypeFIELD", "periodFIELD", "headerfileFIELD",
  "sharedFIELD", "localFIELD", "schemaFIELD", "agentsFIELD", "identifier",
  "headerfl", "integer", "equal", "semicomma", "comma", "openbrace",
  "closebrace", "eol", "eof", "$accept", "S", "INITIAL", "$@1", "$@2",
  "$@3", "$@4", "$@5", "AGENTS", "ITEMOPEN", "$@6", "ITEM", "$@7", "$@8",
  "$@9", "$@10", "ITEMAFTERFIELD", "$@11", "SCHEMAOPEN", "$@12", "SCHEMA",
  "$@13", "SHAREDITEMS", "opt_eol", "LOCALITEMS", "ASSIGNMENTOPEN", "$@14",
  "$@15", "ASSIGNMENT", "$@16", "$@17", "$@18", "ASSIGNMENTAGENTS", 0
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[YYLEX-NUM] -- Internal token number corresponding to
   token YYLEX-NUM.  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,    24,    25,    27,    26,    28,    26,    29,    26,    30,
      26,    31,    26,    26,    26,    26,    32,    32,    32,    34,
      33,    33,    33,    36,    35,    37,    35,    38,    35,    39,
      35,    35,    35,    41,    40,    40,    40,    40,    43,    42,
      42,    42,    45,    44,    44,    44,    44,    44,    44,    44,
      46,    46,    46,    47,    47,    48,    48,    48,    50,    49,
      51,    49,    49,    53,    52,    54,    52,    55,    52,    52,
      52,    52,    52,    56,    56,    56
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     1,     0,     3,     0,     6,     0,     7,     0,
       5,     0,     5,     3,     1,     1,     1,     3,     1,     0,
       3,     2,     1,     0,     3,     0,     5,     0,     5,     0,
       5,     1,     1,     0,     3,     2,     1,     1,     0,     3,
       2,     1,     0,     3,     5,     4,     5,     4,     1,     1,
       1,     4,     1,     0,     1,     1,     4,     1,     0,     3,
       0,     3,     1,     0,     3,     0,     6,     0,     5,     5,
       4,     1,     1,     1,     3,     1
};

/* YYDEFACT[STATE-NAME] -- Default reduction number in state STATE-NUM.
   Performed when YYTABLE doesn't specify something else to do.  Zero
   means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
       0,    15,     0,     0,     0,     0,     3,    14,     0,     2,
       0,     9,    11,    62,    60,    58,     0,     0,     1,    18,
      16,     0,     0,     0,     0,     0,    13,     4,     0,     0,
       5,    22,     0,    19,     0,    41,     0,    38,     0,    72,
       0,     0,    71,    63,    61,    59,     7,    17,     0,    32,
       0,     0,     0,    31,    23,    21,     0,    10,    49,     0,
       0,    48,    42,    40,     0,    12,     0,     0,     0,     0,
       6,     0,     0,     0,     0,    20,     0,     0,     0,    39,
      67,    75,    73,     0,    64,     8,    25,    27,    29,    24,
      52,    50,     0,    57,    55,     0,    43,     0,     0,     0,
       0,    70,     0,     0,     0,     0,    53,    45,     0,    53,
      47,     0,    68,    69,    74,    37,     0,    36,    33,    26,
      28,    30,    44,    54,     0,    46,     0,    66,    35,     0,
      51,    56,    34
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
      -1,     8,     9,    17,    48,    69,    22,    23,    21,    34,
      56,    55,    74,   102,   103,   104,   119,   129,    38,    64,
      63,    78,    92,   124,    95,    16,    25,    24,    44,    68,
      97,    98,    83
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
#define YYPACT_NINF -75
static const yytype_int8 yypact[] =
{
       5,   -75,    15,     8,    24,    61,   -75,   -75,    39,   -75,
       2,   -75,   -75,   -75,   -75,   -75,     5,     5,   -75,   -75,
     -75,    -4,    79,    90,    66,    61,   -75,   -75,    22,    31,
     -75,   -75,    50,   -75,     5,   -75,    74,   -75,     5,   -75,
      51,    53,   -75,   -75,   -75,   -75,   -75,   -75,     5,   -75,
      65,    75,    76,   -75,   -75,   -75,    79,   -75,   -75,    77,
      80,   -75,   -75,   -75,    90,   -75,    84,     3,    66,     5,
     -75,    86,    88,    89,    50,   -75,    11,    12,    74,   -75,
      85,   -75,   -75,    28,   -75,   -75,   -75,   -75,   -75,   -75,
     -75,   -75,    42,   -75,   -75,    55,   -75,    87,    66,    66,
      92,   -75,    68,    68,    68,    74,    91,   -75,    74,    91,
     -75,    66,   -75,   -75,   -75,   -75,    50,   -75,   -75,   -75,
     -75,   -75,   -75,   -75,    93,   -75,    94,   -75,   -75,    50,
     -75,   -75,   -75
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int8 yypgoto[] =
{
     -75,   -75,   -15,   -75,   -75,   -75,   -75,   -75,   -75,    58,
     -75,   -74,   -75,   -75,   -75,   -75,   -73,   -75,    45,   -75,
     -71,   -75,   -75,     6,   -75,    95,   -75,   -75,   -63,   -75,
     -75,   -75,   -75
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule which
   number is the opposite.  If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -66
static const yytype_int16 yytable[] =
{
      89,    26,    27,    19,    81,    84,     1,    96,     2,     3,
       4,     5,    90,    93,    28,    29,    20,    82,    30,    57,
     101,   107,    11,    65,   110,    91,    94,     6,     7,    39,
     120,   121,    10,    70,   122,   112,   113,   125,    12,    18,
      40,    41,   128,    58,    46,    47,    99,   100,   127,    42,
      43,    49,    59,    60,    85,   132,    58,    50,    51,    52,
     105,   106,    13,    61,    62,    59,    60,    39,    66,   115,
      67,    53,    54,   108,   109,    58,    61,    62,    40,    41,
      31,    14,    71,    15,    59,    60,   116,    42,    43,   117,
     118,    35,    72,    73,    76,    61,    62,    77,    80,    32,
      86,    33,    87,   -65,    88,   111,   114,   130,   131,    79,
      36,     0,    37,   123,    75,   126,     0,     0,     0,     0,
      45
};

#define yypact_value_is_default(yystate) \
  ((yystate) == (-75))

#define yytable_value_is_error(yytable_value) \
  YYID (0)

static const yytype_int16 yycheck[] =
{
      74,    16,    17,     1,     1,    68,     1,    78,     3,     4,
       5,     6,     1,     1,    18,    19,    14,    14,    22,    34,
      83,    92,    14,    38,    95,    14,    14,    22,    23,     1,
     103,   104,    17,    48,   105,    98,    99,   108,    14,     0,
      12,    13,   116,     1,    22,    14,    18,    19,   111,    21,
      22,     1,    10,    11,    69,   129,     1,     7,     8,     9,
      18,    19,     1,    21,    22,    10,    11,     1,    17,     1,
      17,    21,    22,    18,    19,     1,    21,    22,    12,    13,
       1,    20,    17,    22,    10,    11,    18,    21,    22,    21,
      22,     1,    17,    17,    17,    21,    22,    17,    14,    20,
      14,    22,    14,    18,    15,    18,    14,    14,    14,    64,
      20,    -1,    22,    22,    56,   109,    -1,    -1,    -1,    -1,
      25
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
   symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,     1,     3,     4,     5,     6,    22,    23,    25,    26,
      17,    14,    14,     1,    20,    22,    49,    27,     0,     1,
      14,    32,    30,    31,    51,    50,    26,    26,    18,    19,
      22,     1,    20,    22,    33,     1,    20,    22,    42,     1,
      12,    13,    21,    22,    52,    49,    22,    14,    28,     1,
       7,     8,     9,    21,    22,    35,    34,    26,     1,    10,
      11,    21,    22,    44,    43,    26,    17,    17,    53,    29,
      26,    17,    17,    17,    36,    33,    17,    17,    45,    42,
      14,     1,    14,    56,    52,    26,    14,    14,    15,    35,
       1,    14,    46,     1,    14,    48,    44,    54,    55,    18,
      19,    52,    37,    38,    39,    18,    19,    44,    18,    19,
      44,    18,    52,    52,    14,     1,    18,    21,    22,    40,
      40,    40,    44,    22,    47,    44,    47,    52,    35,    41,
      14,    14,    35
};

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		(-2)
#define YYEOF		0

#define YYACCEPT	goto yyacceptlab
#define YYABORT		goto yyabortlab
#define YYERROR		goto yyerrorlab


/* Like YYERROR except do call yyerror.  This remains here temporarily
   to ease the transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  However,
   YYFAIL appears to be in use.  Nevertheless, it is formally deprecated
   in Bison 2.4.2's NEWS entry, where a plan to phase it out is
   discussed.  */

#define YYFAIL		goto yyerrlab
#if defined YYFAIL
  /* This is here to suppress warnings from the GCC cpp's
     -Wunused-macros.  Normally we don't worry about that warning, but
     some users do, and we want to make it easy for users to remove
     YYFAIL uses, which will produce warnings from Bison 2.5.  */
#endif

#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)					\
do								\
  if (yychar == YYEMPTY && yylen == 1)				\
    {								\
      yychar = (Token);						\
      yylval = (Value);						\
      YYPOPSTACK (1);						\
      goto yybackup;						\
    }								\
  else								\
    {								\
      yyerror (YY_("syntax error: cannot back up")); \
      YYERROR;							\
    }								\
while (YYID (0))


#define YYTERROR	1
#define YYERRCODE	256


/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

#define YYRHSLOC(Rhs, K) ((Rhs)[K])
#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)				\
    do									\
      if (YYID (N))                                                    \
	{								\
	  (Current).first_line   = YYRHSLOC (Rhs, 1).first_line;	\
	  (Current).first_column = YYRHSLOC (Rhs, 1).first_column;	\
	  (Current).last_line    = YYRHSLOC (Rhs, N).last_line;		\
	  (Current).last_column  = YYRHSLOC (Rhs, N).last_column;	\
	}								\
      else								\
	{								\
	  (Current).first_line   = (Current).last_line   =		\
	    YYRHSLOC (Rhs, 0).last_line;				\
	  (Current).first_column = (Current).last_column =		\
	    YYRHSLOC (Rhs, 0).last_column;				\
	}								\
    while (YYID (0))
#endif


/* This macro is provided for backward compatibility. */

#ifndef YY_LOCATION_PRINT
# define YY_LOCATION_PRINT(File, Loc) ((void) 0)
#endif


/* YYLEX -- calling `yylex' with the right arguments.  */

#ifdef YYLEX_PARAM
# define YYLEX yylex (&yylval, YYLEX_PARAM)
#else
# define YYLEX yylex (&yylval)
#endif

/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)			\
do {						\
  if (yydebug)					\
    YYFPRINTF Args;				\
} while (YYID (0))

# define YY_SYMBOL_PRINT(Title, Type, Value, Location)			  \
do {									  \
  if (yydebug)								  \
    {									  \
      YYFPRINTF (stderr, "%s ", Title);					  \
      yy_symbol_print (stderr,						  \
		  Type, Value); \
      YYFPRINTF (stderr, "\n");						  \
    }									  \
} while (YYID (0))


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_value_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# else
  YYUSE (yyoutput);
# endif
  switch (yytype)
    {
      default:
	break;
    }
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (yytype < YYNTOKENS)
    YYFPRINTF (yyoutput, "token %s (", yytname[yytype]);
  else
    YYFPRINTF (yyoutput, "nterm %s (", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_stack_print (yytype_int16 *yybottom, yytype_int16 *yytop)
#else
static void
yy_stack_print (yybottom, yytop)
    yytype_int16 *yybottom;
    yytype_int16 *yytop;
#endif
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)				\
do {								\
  if (yydebug)							\
    yy_stack_print ((Bottom), (Top));				\
} while (YYID (0))


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_reduce_print (YYSTYPE *yyvsp, int yyrule)
#else
static void
yy_reduce_print (yyvsp, yyrule)
    YYSTYPE *yyvsp;
    int yyrule;
#endif
{
  int yynrhs = yyr2[yyrule];
  int yyi;
  unsigned long int yylno = yyrline[yyrule];
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
	     yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr, yyrhs[yyprhs[yyrule] + yyi],
		       &(yyvsp[(yyi + 1) - (yynrhs)])
		       		       );
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)		\
do {					\
  if (yydebug)				\
    yy_reduce_print (yyvsp, Rule); \
} while (YYID (0))

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef	YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif


#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static YYSIZE_T
yystrlen (const char *yystr)
#else
static YYSIZE_T
yystrlen (yystr)
    const char *yystr;
#endif
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static char *
yystpcpy (char *yydest, const char *yysrc)
#else
static char *
yystpcpy (yydest, yysrc)
    char *yydest;
    const char *yysrc;
#endif
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
	switch (*++yyp)
	  {
	  case '\'':
	  case ',':
	    goto do_not_strip_quotes;

	  case '\\':
	    if (*++yyp != '\\')
	      goto do_not_strip_quotes;
	    /* Fall through.  */
	    if (yyres)
		  yyres[yyn] = *yyp;
		yyn++;
		break;
	  default:
	    if (yyres)
	      yyres[yyn] = *yyp;
	    yyn++;
	    break;

	  case '"':
	    if (yyres)
	      yyres[yyn] = '\0';
	    return yyn;
	  }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into *YYMSG, which is of size *YYMSG_ALLOC, an error message
   about the unexpected token YYTOKEN for the state stack whose top is
   YYSSP.

   Return 0 if *YYMSG was successfully written.  Return 1 if *YYMSG is
   not large enough to hold the message.  In that case, also set
   *YYMSG_ALLOC to the required number of bytes.  Return 2 if the
   required number of bytes is too large to store.  */
static int
yysyntax_error (YYSIZE_T *yymsg_alloc, char **yymsg,
                yytype_int16 *yyssp, int yytoken)
{
  YYSIZE_T yysize0 = yytnamerr (0, yytname[yytoken]);
  YYSIZE_T yysize = yysize0;
  YYSIZE_T yysize1;
  enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
  /* Internationalized format string. */
  const char *yyformat = 0;
  /* Arguments of yyformat. */
  char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
  /* Number of reported tokens (one for the "unexpected", one per
     "expected"). */
  int yycount = 0;

  /* There are many possibilities here to consider:
     - Assume YYFAIL is not used.  It's too flawed to consider.  See
       <http://lists.gnu.org/archive/html/bison-patches/2009-12/msg00024.html>
       for details.  YYERROR is fine as it does not invoke this
       function.
     - If this state is a consistent state with a default action, then
       the only way this function was invoked is if the default action
       is an error action.  In that case, don't check for expected
       tokens because there are none.
     - The only way there can be no lookahead present (in yychar) is if
       this state is a consistent state with a default action.  Thus,
       detecting the absence of a lookahead is sufficient to determine
       that there is no unexpected or expected token to report.  In that
       case, just report a simple "syntax error".
     - Don't assume there isn't a lookahead just because this state is a
       consistent state with a default action.  There might have been a
       previous inconsistent state, consistent state with a non-default
       action, or user semantic action that manipulated yychar.
     - Of course, the expected token list depends on states to have
       correct lookahead information, and it depends on the parser not
       to perform extra reductions after fetching a lookahead from the
       scanner and before detecting a syntax error.  Thus, state merging
       (from LALR or IELR) and default reductions corrupt the expected
       token list.  However, the list is correct for canonical LR with
       one exception: it will still contain any token that will not be
       accepted due to an error action in a later state.
  */
  if (yytoken != YYEMPTY)
    {
      int yyn = yypact[*yyssp];
      yyarg[yycount++] = yytname[yytoken];
      if (!yypact_value_is_default (yyn))
        {
          /* Start YYX at -YYN if negative to avoid negative indexes in
             YYCHECK.  In other words, skip the first -YYN actions for
             this state because they are default actions.  */
          int yyxbegin = yyn < 0 ? -yyn : 0;
          /* Stay within bounds of both yycheck and yytname.  */
          int yychecklim = YYLAST - yyn + 1;
          int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
          int yyx;

          for (yyx = yyxbegin; yyx < yyxend; ++yyx)
            if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR
                && !yytable_value_is_error (yytable[yyx + yyn]))
              {
                if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
                  {
                    yycount = 1;
                    yysize = yysize0;
                    break;
                  }
                yyarg[yycount++] = yytname[yyx];
                yysize1 = yysize + yytnamerr (0, yytname[yyx]);
                if (! (yysize <= yysize1
                       && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
                  return 2;
                yysize = yysize1;
              }
        }
    }

  switch (yycount)
    {
# define YYCASE_(N, S)                      \
      case N:                               \
        yyformat = S;                       \
      break
      YYCASE_(0, YY_("syntax error"));
      YYCASE_(1, YY_("syntax error, unexpected %s"));
      YYCASE_(2, YY_("syntax error, unexpected %s, expecting %s"));
      YYCASE_(3, YY_("syntax error, unexpected %s, expecting %s or %s"));
      YYCASE_(4, YY_("syntax error, unexpected %s, expecting %s or %s or %s"));
      YYCASE_(5, YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s"));
# undef YYCASE_
    }

  yysize1 = yysize + yystrlen (yyformat);
  if (! (yysize <= yysize1 && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
    return 2;
  yysize = yysize1;

  if (*yymsg_alloc < yysize)
    {
      *yymsg_alloc = 2 * yysize;
      if (! (yysize <= *yymsg_alloc
             && *yymsg_alloc <= YYSTACK_ALLOC_MAXIMUM))
        *yymsg_alloc = YYSTACK_ALLOC_MAXIMUM;
      return 1;
    }

  /* Avoid sprintf, as that infringes on the user's name space.
     Don't have undefined behavior even if the translation
     produced a string with the wrong number of "%s"s.  */
  {
    char *yyp = *yymsg;
    int yyi = 0;
    while ((*yyp = *yyformat) != '\0')
      if (*yyp == '%' && yyformat[1] == 's' && yyi < yycount)
        {
          yyp += yytnamerr (yyp, yyarg[yyi++]);
          yyformat += 2;
        }
      else
        {
          yyp++;
          yyformat++;
        }
  }
  return 0;
}
#endif /* YYERROR_VERBOSE */

/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep)
#else
static void
yydestruct (yymsg, yytype, yyvaluep)
    const char *yymsg;
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  YYUSE (yyvaluep);

  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  switch (yytype)
    {

      default:
	break;
    }
}


/* Prevent warnings from -Wmissing-prototypes.  */
#ifdef YYPARSE_PARAM
#if defined __STDC__ || defined __cplusplus
int yyparse (void *YYPARSE_PARAM);
#else
int yyparse ();
#endif
#else /* ! YYPARSE_PARAM */
#if defined __STDC__ || defined __cplusplus
int yyparse (void);
#else
int yyparse ();
#endif
#endif /* ! YYPARSE_PARAM */


/*----------.
| yyparse.  |
`----------*/

#ifdef YYPARSE_PARAM
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void *YYPARSE_PARAM)
#else
int
yyparse (YYPARSE_PARAM)
    void *YYPARSE_PARAM;
#endif
#else /* ! YYPARSE_PARAM */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void)
#else
int
yyparse ()

#endif
#endif
{
/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;

    /* Number of syntax errors so far.  */
    int yynerrs;

    int yystate;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus;

    /* The stacks and their tools:
       `yyss': related to states.
       `yyvs': related to semantic values.

       Refer to the stacks thru separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* The state stack.  */
    yytype_int16 yyssa[YYINITDEPTH];
    yytype_int16 *yyss;
    yytype_int16 *yyssp;

    /* The semantic value stack.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs;
    YYSTYPE *yyvsp;

    YYSIZE_T yystacksize;

  int yyn;
  int yyresult;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;

#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  yytoken = 0;
  yyss = yyssa;
  yyvs = yyvsa;
  yystacksize = YYINITDEPTH;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY; /* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */
  yyssp = yyss;
  yyvsp = yyvs;

  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
	/* Give user a chance to reallocate the stack.  Use copies of
	   these so that the &'s don't force the real ones into
	   memory.  */
	YYSTYPE *yyvs1 = yyvs;
	yytype_int16 *yyss1 = yyss;

	/* Each stack pointer address is followed by the size of the
	   data in use in that stack, in bytes.  This used to be a
	   conditional around just the two extra args, but that might
	   be undefined if yyoverflow is a macro.  */
	yyoverflow (YY_("memory exhausted"),
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),
		    &yystacksize);

	yyss = yyss1;
	yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
	goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
	yystacksize = YYMAXDEPTH;

      {
	yytype_int16 *yyss1 = yyss;
	union yyalloc *yyptr =
	  (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
	if (! yyptr)
	  goto yyexhaustedlab;
	YYSTACK_RELOCATE (yyss_alloc, yyss);
	YYSTACK_RELOCATE (yyvs_alloc, yyvs);
#  undef YYSTACK_RELOCATE
	if (yyss1 != yyssa)
	  YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;

      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
		  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
	YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to lookahead token.  */
  yyn = yypact[yystate];
  if (yypact_value_is_default (yyn))
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid lookahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = YYLEX;
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yytable_value_is_error (yyn))
        goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token.  */
  yychar = YYEMPTY;

  yystate = yyn;
  *++yyvsp = yylval;

  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     `$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 3:

/* Line 1806 of yacc.c  */
#line 50 "xrtdb.y"
    { nline++; }
    break;

  case 5:

/* Line 1806 of yacc.c  */
#line 51 "xrtdb.y"
    { nline++; }
    break;

  case 7:

/* Line 1806 of yacc.c  */
#line 52 "xrtdb.y"
    { nline++; }
    break;

  case 9:

/* Line 1806 of yacc.c  */
#line 53 "xrtdb.y"
    { pItem= itemCreate((yyvsp[(2) - (2)])); }
    break;

  case 11:

/* Line 1806 of yacc.c  */
#line 54 "xrtdb.y"
    { pSchema= schemaCreate((yyvsp[(2) - (2)])); }
    break;

  case 14:

/* Line 1806 of yacc.c  */
#line 56 "xrtdb.y"
    { return 0; }
    break;

  case 15:

/* Line 1806 of yacc.c  */
#line 57 "xrtdb.y"
    { raiseError(nline, _ERR_INITIAL_); }
    break;

  case 16:

/* Line 1806 of yacc.c  */
#line 60 "xrtdb.y"
    { agentCreate((yyvsp[(1) - (1)])); }
    break;

  case 17:

/* Line 1806 of yacc.c  */
#line 61 "xrtdb.y"
    { agentCreate((yyvsp[(3) - (3)])); }
    break;

  case 18:

/* Line 1806 of yacc.c  */
#line 62 "xrtdb.y"
    { raiseError(nline, _ERR_AGENTS_); }
    break;

  case 19:

/* Line 1806 of yacc.c  */
#line 65 "xrtdb.y"
    { nline++; }
    break;

  case 22:

/* Line 1806 of yacc.c  */
#line 67 "xrtdb.y"
    { raiseError(nline, _ERR_ITEMOPEN_); }
    break;

  case 23:

/* Line 1806 of yacc.c  */
#line 70 "xrtdb.y"
    { nline++; }
    break;

  case 25:

/* Line 1806 of yacc.c  */
#line 71 "xrtdb.y"
    { itemAddDatatype(pItem, (yyvsp[(3) - (3)])); }
    break;

  case 27:

/* Line 1806 of yacc.c  */
#line 72 "xrtdb.y"
    { itemAddPeriod(pItem, (yyvsp[(3) - (3)])); }
    break;

  case 29:

/* Line 1806 of yacc.c  */
#line 73 "xrtdb.y"
    { itemAddHeaderfile(pItem, (yyvsp[(3) - (3)])); }
    break;

  case 31:

/* Line 1806 of yacc.c  */
#line 74 "xrtdb.y"
    { itemVerify(pItem); }
    break;

  case 32:

/* Line 1806 of yacc.c  */
#line 75 "xrtdb.y"
    { raiseError(nline, _ERR_ITEMFIELD_); }
    break;

  case 33:

/* Line 1806 of yacc.c  */
#line 78 "xrtdb.y"
    { nline++; }
    break;

  case 36:

/* Line 1806 of yacc.c  */
#line 80 "xrtdb.y"
    { itemVerify(pItem); }
    break;

  case 37:

/* Line 1806 of yacc.c  */
#line 81 "xrtdb.y"
    { raiseError(nline, _ERR_ITEMAFTERFIELD_); }
    break;

  case 38:

/* Line 1806 of yacc.c  */
#line 84 "xrtdb.y"
    { nline++; }
    break;

  case 41:

/* Line 1806 of yacc.c  */
#line 86 "xrtdb.y"
    { raiseError(nline, _ERR_SCHEMAOPEN_); }
    break;

  case 42:

/* Line 1806 of yacc.c  */
#line 89 "xrtdb.y"
    { nline++; }
    break;

  case 48:

/* Line 1806 of yacc.c  */
#line 94 "xrtdb.y"
    { schemaVerify(pSchema); }
    break;

  case 49:

/* Line 1806 of yacc.c  */
#line 95 "xrtdb.y"
    { raiseError(nline, _ERR_SCHEMAFIELD_); }
    break;

  case 50:

/* Line 1806 of yacc.c  */
#line 98 "xrtdb.y"
    { schemaAddSharedItem(pSchema, (yyvsp[(1) - (1)])); }
    break;

  case 51:

/* Line 1806 of yacc.c  */
#line 99 "xrtdb.y"
    { schemaAddSharedItem(pSchema, (yyvsp[(4) - (4)])); }
    break;

  case 52:

/* Line 1806 of yacc.c  */
#line 100 "xrtdb.y"
    { raiseError(nline, _ERR_ITEMSLIST_); }
    break;

  case 54:

/* Line 1806 of yacc.c  */
#line 104 "xrtdb.y"
    { nline++; }
    break;

  case 55:

/* Line 1806 of yacc.c  */
#line 107 "xrtdb.y"
    { schemaAddLocalItem(pSchema, (yyvsp[(1) - (1)])); }
    break;

  case 56:

/* Line 1806 of yacc.c  */
#line 108 "xrtdb.y"
    { schemaAddLocalItem(pSchema, (yyvsp[(4) - (4)])); }
    break;

  case 57:

/* Line 1806 of yacc.c  */
#line 109 "xrtdb.y"
    { raiseError(nline, _ERR_ITEMSLIST_); }
    break;

  case 58:

/* Line 1806 of yacc.c  */
#line 112 "xrtdb.y"
    { nline++; }
    break;

  case 60:

/* Line 1806 of yacc.c  */
#line 113 "xrtdb.y"
    { pAssign= assignmentCreate(); }
    break;

  case 62:

/* Line 1806 of yacc.c  */
#line 114 "xrtdb.y"
    { raiseError(nline, _ERR_ASSIGNMENTOPEN_); }
    break;

  case 63:

/* Line 1806 of yacc.c  */
#line 117 "xrtdb.y"
    { nline++; }
    break;

  case 65:

/* Line 1806 of yacc.c  */
#line 118 "xrtdb.y"
    { assignmentAddSchema(pAssign, (yyvsp[(3) - (3)])); }
    break;

  case 67:

/* Line 1806 of yacc.c  */
#line 119 "xrtdb.y"
    { assignmentAddSchema(pAssign, (yyvsp[(3) - (3)])); }
    break;

  case 71:

/* Line 1806 of yacc.c  */
#line 122 "xrtdb.y"
    { assignmentVerify(pAssign); }
    break;

  case 72:

/* Line 1806 of yacc.c  */
#line 123 "xrtdb.y"
    { raiseError(nline, _ERR_ASSIGNMENT_); }
    break;

  case 73:

/* Line 1806 of yacc.c  */
#line 126 "xrtdb.y"
    { assignmentAddAgent(pAssign, (yyvsp[(1) - (1)])); }
    break;

  case 74:

/* Line 1806 of yacc.c  */
#line 127 "xrtdb.y"
    { assignmentAddAgent(pAssign, (yyvsp[(3) - (3)])); }
    break;

  case 75:

/* Line 1806 of yacc.c  */
#line 128 "xrtdb.y"
    { raiseError(nline, _ERR_AGENTSLIST_); }
    break;



/* Line 1806 of yacc.c  */
#line 1809 "xrtdb.tab.c"
      default: break;
    }
  /* User semantic actions sometimes alter yychar, and that requires
     that yytoken be updated with the new translation.  We take the
     approach of translating immediately before every use of yytoken.
     One alternative is translating here after every semantic action,
     but that translation would be missed if the semantic action invokes
     YYABORT, YYACCEPT, or YYERROR immediately after altering yychar or
     if it invokes YYBACKUP.  In the case of YYABORT or YYACCEPT, an
     incorrect destructor might then be invoked immediately.  In the
     case of YYERROR or YYBACKUP, subsequent parser actions might lead
     to an incorrect destructor call or verbose syntax error message
     before the lookahead is translated.  */
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;

  /* Now `shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*------------------------------------.
| yyerrlab -- here on detecting error |
`------------------------------------*/
yyerrlab:
  /* Make sure we have latest lookahead translation.  See comments at
     user semantic actions for why this is necessary.  */
  yytoken = yychar == YYEMPTY ? YYEMPTY : YYTRANSLATE (yychar);

  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (YY_("syntax error"));
#else
# define YYSYNTAX_ERROR yysyntax_error (&yymsg_alloc, &yymsg, \
                                        yyssp, yytoken)
      {
        char const *yymsgp = YY_("syntax error");
        int yysyntax_error_status;
        yysyntax_error_status = YYSYNTAX_ERROR;
        if (yysyntax_error_status == 0)
          yymsgp = yymsg;
        else if (yysyntax_error_status == 1)
          {
            if (yymsg != yymsgbuf)
              YYSTACK_FREE (yymsg);
            yymsg = (char *) YYSTACK_ALLOC (yymsg_alloc);
            if (!yymsg)
              {
                yymsg = yymsgbuf;
                yymsg_alloc = sizeof yymsgbuf;
                yysyntax_error_status = 2;
              }
            else
              {
                yysyntax_error_status = YYSYNTAX_ERROR;
                yymsgp = yymsg;
              }
          }
        yyerror (yymsgp);
        if (yysyntax_error_status == 2)
          goto yyexhaustedlab;
      }
# undef YYSYNTAX_ERROR
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
	 error, discard it.  */

      if (yychar <= YYEOF)
	{
	  /* Return failure if at end of input.  */
	  if (yychar == YYEOF)
	    YYABORT;
	}
      else
	{
	  yydestruct ("Error: discarding",
		      yytoken, &yylval);
	  yychar = YYEMPTY;
	}
    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule which action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;	/* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (!yypact_value_is_default (yyn))
	{
	  yyn += YYTERROR;
	  if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
	    {
	      yyn = yytable[yyn];
	      if (0 < yyn)
		break;
	    }
	}

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
	YYABORT;


      yydestruct ("Error: popping",
		  yystos[yystate], yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  *++yyvsp = yylval;


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#if !defined(yyoverflow) || YYERROR_VERBOSE
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEMPTY)
    {
      /* Make sure we have latest lookahead translation.  See comments at
         user semantic actions for why this is necessary.  */
      yytoken = YYTRANSLATE (yychar);
      yydestruct ("Cleanup: discarding lookahead",
                  yytoken, &yylval);
    }
  /* Do not reclaim the symbols of the rule which action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
		  yystos[*yyssp], yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  /* Make sure YYID is used.  */
  return YYID (yyresult);
}



/* Line 2067 of yacc.c  */
#line 131 "xrtdb.y"



/* Main function -> Entry point for the program */
int main(int argc, char *argv[])
{
        //If the program wasn't correctly called, abort and teach correct usage
        //Include the reference to the file pointer used by flex
        extern FILE *yyin;

        if (argc != 2)
        {
				printf("*** Using default configuration file ***\n");
                printf("To use other configuration file: $ %s fich_conf\n", *argv);
				//If the input file doesn't exist or there's no reading permissions, abort
				if (!( (yyin= fopen(RTDB_CONF, "r")) != NULL ))
				{
						printf("Erro na abertura do ficheiro de configuração!\nNão foi possível abrir \"%s\" para leitura\n", argv[1]);
						return 1;
				}

        }
		else
		{
				//If the input file doesn't exist or there's no reading permissions, abort
				if (!( (yyin= fopen(argv[1], "r")) != NULL ))
				{
						printf("Erro na abertura do ficheiro de configuração!\nNão foi possível abrir \"%s\" para leitura\n", argv[1]);
						return 1;
				}
		}

        // Initialize global lists to store the declared agents, items, schemas and assignments
        agList.numAg= 0;
        agList.agents= malloc(sizeof(rtdb_Agent));

        itList.numIt= 0;
        itList.items= malloc(sizeof(rtdb_Item));

        schemaList.numSc= 0;
        schemaList.schemas= malloc(sizeof(rtdb_Schema));

        assignList.numAs= 0;
        assignList.asList= malloc(sizeof(rtdb_Assignment));

        //Parse the rtdb.conf file
        if (yyparse() == 0)
        {
			if(argc == 2)		
                printf("\nFicheiro \e[33m%s\e[0m lido com sucesso!\n", argv[1]);
			else
                printf("\nFicheiro \e[33m%s\e[0m lido com sucesso!\n", RTDB_CONF);
        }
        else {
                printf("\n\e[33mERRO\e[0m a ler o ficheiro\e[32m%s\e[0m!\n", argv[1]);
                return 1;
             }

        //If in Debug mode, print detailed information about the data stored on memory after reading the input file
        if (DEBUG)
        {
                unsigned i, j;

                printf("\n\n\e[32mA imprimir informação de Debugging detalhada sobre os dados guardados\n   em memória após a leitura do ficheiro \e[33m%s\e[0m\e[32m.\e[0m\n\n", argv[1]);

                //Print Agents List
                printf("\n\e[33mA imprimir a lista de \e[32m%u\e[33m agentes:\e[0m", agList.numAg);
                for (i= 0; i < agList.numAg; i++)
                {
                        printf("\n\e[32mAgent:\e[0m %u: \e[32mID:\e[0m %s", agList.agents[i].num, agList.agents[i].id);
                }
                printf("\n\e[33mFim da lista de agentes.\e[0m\n");

                //Print Items list
                printf("\n\e[33mA imprimir a lista de \e[32m%u\e[33m items:\e[0m", itList.numIt);
                for (i= 0; i < itList.numIt; i++)
                {
                        printf("\n\e[32mItem\e[0m %u: \e[32mID:\e[0m %s \e[32mDatatype:\e[0m %s \e[32mHeaderfile:\e[0m %s \e[32mPeriod:\e[0m %u", itList.items[i].num, itList.items[i].id, itList.items[i].datatype, itList.items[i].headerfile, itList.items[i].period);
                }
                printf("\n\e[33mFim da lista de items.\e[0m\n");

                //Print Schemas List
                printf("\n\e[33mA imprimir a lista de \e[32m%u\e[33m Esquemas:\e[0m", schemaList.numSc);
                for (i= 0; i < schemaList.numSc; i++)
                {
                        printf("\n\e[33mA imprimir lista de \e[32m%u\e[33m sharedItems do esquema \e[32m%s\e[33m na posicao \e[32m%u\e[33m da lista de esquemas:\e[0m", schemaList.schemas[i].sharedItems.numIt, schemaList.schemas[i].id, i);
                        //Print shared items list
                        for (j= 0; j < schemaList.schemas[i].sharedItems.numIt; j++)
                        {
                                printf("\n\e[32mItem\e[0m %u: \e[32mID:\e[0m %s \e[32mDatatype:\e[0m %s \e[32mHeaderfile:\e[0m %s \e[32mPeriod:\e[0m %u", schemaList.schemas[i].sharedItems.items[j].num, schemaList.schemas[i].sharedItems.items[j].id, schemaList.schemas[i].sharedItems.items[j].datatype, schemaList.schemas[i].sharedItems.items[j].headerfile, schemaList.schemas[i].sharedItems.items[j].period);
                        }
                        printf("\n\e[33mA imprimir lista de \e[32m%u\e[33m local Items do esquema \e[32m%s\e[33m na posicao \e[32m%u\e[33m da lista de esquemas:\e[0m", schemaList.schemas[i].localItems.numIt, schemaList.schemas[i].id, i);
                        //Print local items list
                        for (j= 0; j < schemaList.schemas[i].localItems.numIt; j++)
                        {
                                printf("\n\e[32mItem\e[0m %u: \e[32mID:\e[0m %s \e[32mDatatype:\e[0m %s \e[32mHeaderfile:\e[0m %s \e[32mPeriod:\e[0m %u", schemaList.schemas[i].localItems.items[j].num, schemaList.schemas[i].localItems.items[j].id, schemaList.schemas[i].localItems.items[j].datatype, schemaList.schemas[i].localItems.items[j].headerfile, schemaList.schemas[i].localItems.items[j].period);
                        }
                }
                printf("\n\e[33mFim da lista de Esquemas.\e[0m\n");

                //Print Assignments list        
                printf("\n\e[33mA imprimir lista de \e[32m%u\e[33m Assignments:\e[0m", assignList.numAs);
                for (i= 0; i < assignList.numAs; i++)
                {
                        printf("\n\e[33mO assignment na posição \e[32m%u\e[33m da lista tem o esquema \e[32m%s\e[0m\e[0m", i, assignList.asList[i].schema->id);
                        printf("\n\e[33mA imprimir lista de \e[32m%u\e[33m agentes do assignment:\e[0m", assignList.asList[i].agentList.numAg);
                        for (j= 0; j < assignList.asList[i].agentList.numAg; j++)
                        {
                                printf("\n\e[32mIndex:\e[0m %u \e[32mAgente: \e[0m%u\e[32m ID:\e[0m %s", j, assignList.asList[i].agentList.agents[j].num, assignList.asList[i].agentList.agents[j].id);
                        }
                        printf("\n");
                }
                printf("\e[33mFim da lista de Assignments.\e[0m\n");

                printf("\n\n\e[32mFIM do dump das estruturas guardadas em memória!\e[0m\n\n");

        }

        //Vars to store return values of printUserFile() and printIniFile()
        int userFileStatus, iniFileStatus; 

        printf("\nA criar o ficheiro \e[32mrtdb_user.h\e[0m\n");

        //Call printUserFile() to generate the rtdb_user.h file
        userFileStatus= printUserFile(itList, agList);

        //Check return value of printUserFile() and output according message
        switch (userFileStatus)
        {
                case 0: {
                        printf("\nFicheiro \e[32mrtdb_user.h\e[0m criado com sucesso!\n");
                        break;
                        }
                case 1: {
                        printf("\nCriação do ficheiro \e[32mrtdb_user.h\e[0m cancelada pelo utilizador.\n");
                        break;
                        }
                case 2: {
                        printf("\n\e[33mERRO\e[0m a criar o ficheiro \e[32mrtdb_user.h\e[0m. Não foi possível abrir o ficheiro para escrita.\n");
                        break;
                        }
                default:
                	printf("\n\e[33mERRO\e[0m inesperado a criar o ficheiro \e[32mrtdb_user.h\e[0m!\n");
                	break;
        }

        //Check if there are assignments defined
        if (assignList.numAs == 0)
        {
                abortOnError("Não é possível escrever o ficheiro \e[32mrtdb.ini\e[0m!\nNão foi definido nenhum \e[33mAssignment\e[0m no ficheiro de configuração!");
        }

        printf("\nA criar o ficheiro \e[32mrtdb.ini\e[0m\n");

        //Call printIniFile() to generate the rtdb.ini file
        iniFileStatus= printIniFile(agList, assignList);

        //Check return value of printIniFile() and output according message
        switch (iniFileStatus)
        {
                case 0: {
                        printf("\nFicheiro \e[32mrtdb.ini\e[0m criado com sucesso!\n");
                        break;
                        }
                case 1: {
                        printf("\nCriação do ficheiro \e[32mrtdb.ini\e[0m cancelada pelo utilizador.\n");
                        break;
                        }
                case 2: {
                        printf("\n\e[33mERRO\e[0m a criar o ficheiro \e[32mrtdb.ini\e[0m. Não foi possível abrir o ficheiro para escrita.\n");
                        break;
                        }
                default:
                	printf("\n\e[33mERRO\e[0m inesperado a criar o ficheiro \e[32mrtdb.ini\e[0m!\n");
                	break;
        }

        //Free the dynamically allocated vars
        free(agList.agents); free(itList.items);
        free(schemaList.schemas); free(assignList.asList);

        //Free the file pointer
        free(yyin);

        //End program returning 0, meaning everything is OK!
        return 0;
}

/* Bison built-in error handling function */
void yyerror(char* s)
{
    raiseError(nline, s);
}

/* EOF: xrtdb.y */

