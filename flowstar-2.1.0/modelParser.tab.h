/* A Bison parser, made by GNU Bison 3.8.2.  */

/* Bison interface for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015, 2018-2021 Free Software Foundation,
   Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.  */

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

/* DO NOT RELY ON FEATURES THAT ARE NOT DOCUMENTED in the manual,
   especially those whose name start with YY_ or yy_.  They are
   private implementation details that can be changed or removed.  */

#ifndef YY_YY_MODELPARSER_TAB_H_INCLUDED
# define YY_YY_MODELPARSER_TAB_H_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int yydebug;
#endif

/* Token kinds.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    YYEMPTY = -2,
    YYEOF = 0,                     /* "end of file"  */
    YYerror = 256,                 /* error  */
    YYUNDEF = 257,                 /* "invalid token"  */
    NUM = 258,                     /* NUM  */
    IDENT = 259,                   /* IDENT  */
    STATEVAR = 260,                /* STATEVAR  */
    TMVAR = 261,                   /* TMVAR  */
    TM = 262,                      /* TM  */
    EQ = 263,                      /* EQ  */
    GEQ = 264,                     /* GEQ  */
    LEQ = 265,                     /* LEQ  */
    ASSIGN = 266,                  /* ASSIGN  */
    END = 267,                     /* END  */
    MODE = 268,                    /* MODE  */
    INIT = 269,                    /* INIT  */
    BELONGSTO = 270,               /* BELONGSTO  */
    POLYODE1 = 271,                /* POLYODE1  */
    POLYODE2 = 272,                /* POLYODE2  */
    POLYODE3 = 273,                /* POLYODE3  */
    VISUALIZE = 274,               /* VISUALIZE  */
    PARAAGGREG = 275,              /* PARAAGGREG  */
    INTAGGREG = 276,               /* INTAGGREG  */
    TMAGGREG = 277,                /* TMAGGREG  */
    OUTPUT = 278,                  /* OUTPUT  */
    NOOUTPUT = 279,                /* NOOUTPUT  */
    CONTINUOUS = 280,              /* CONTINUOUS  */
    HYBRID = 281,                  /* HYBRID  */
    SETTING = 282,                 /* SETTING  */
    FIXEDST = 283,                 /* FIXEDST  */
    FIXEDORD = 284,                /* FIXEDORD  */
    ADAPTIVEST = 285,              /* ADAPTIVEST  */
    ADAPTIVEORD = 286,             /* ADAPTIVEORD  */
    ORDER = 287,                   /* ORDER  */
    MIN = 288,                     /* MIN  */
    MAX = 289,                     /* MAX  */
    REMEST = 290,                  /* REMEST  */
    INTERVAL = 291,                /* INTERVAL  */
    OCTAGON = 292,                 /* OCTAGON  */
    GRID = 293,                    /* GRID  */
    PLOT = 294,                    /* PLOT  */
    QRPRECOND = 295,               /* QRPRECOND  */
    IDPRECOND = 296,               /* IDPRECOND  */
    TIME = 297,                    /* TIME  */
    MODES = 298,                   /* MODES  */
    JUMPS = 299,                   /* JUMPS  */
    INV = 300,                     /* INV  */
    GUARD = 301,                   /* GUARD  */
    RESET = 302,                   /* RESET  */
    START = 303,                   /* START  */
    MAXJMPS = 304,                 /* MAXJMPS  */
    PRINTON = 305,                 /* PRINTON  */
    PRINTOFF = 306,                /* PRINTOFF  */
    UNSAFESET = 307,               /* UNSAFESET  */
    CONTINUOUSFLOW = 308,          /* CONTINUOUSFLOW  */
    HYBRIDFLOW = 309,              /* HYBRIDFLOW  */
    TAYLOR_PICARD = 310,           /* TAYLOR_PICARD  */
    TAYLOR_REMAINDER = 311,        /* TAYLOR_REMAINDER  */
    TAYLOR_POLYNOMIAL = 312,       /* TAYLOR_POLYNOMIAL  */
    NONPOLY_CENTER = 313,          /* NONPOLY_CENTER  */
    EXP = 314,                     /* EXP  */
    SIN = 315,                     /* SIN  */
    COS = 316,                     /* COS  */
    LOG = 317,                     /* LOG  */
    SQRT = 318,                    /* SQRT  */
    NPODE_TAYLOR = 319,            /* NPODE_TAYLOR  */
    CUTOFF = 320,                  /* CUTOFF  */
    PRECISION = 321,               /* PRECISION  */
    GNUPLOT = 322,                 /* GNUPLOT  */
    MATLAB = 323,                  /* MATLAB  */
    COMPUTATIONPATHS = 324,        /* COMPUTATIONPATHS  */
    LTI_ODE = 325,                 /* LTI_ODE  */
    LTV_ODE = 326,                 /* LTV_ODE  */
    PAR = 327,                     /* PAR  */
    UNC = 328,                     /* UNC  */
    UNIVARIATE_POLY = 329,         /* UNIVARIATE_POLY  */
    TIME_INV = 330,                /* TIME_INV  */
    TIME_VAR = 331,                /* TIME_VAR  */
    STEP = 332,                    /* STEP  */
    TRUE = 333,                    /* TRUE  */
    FALSE = 334,                   /* FALSE  */
    LINEARCONTINUOUSFLOW = 335,    /* LINEARCONTINUOUSFLOW  */
    uminus = 336                   /* uminus  */
  };
  typedef enum yytokentype yytoken_kind_t;
#endif

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
union YYSTYPE
{
#line 61 "modelParser.y"

	double dblVal;
	int intVal;
	std::string *identifier;
	std::vector<flowstar::Interval> *intVec;
	std::vector<int> *iVec;
	std::vector<double> *dVec;
	std::vector<flowstar::Monomial> *monoVec;
	std::vector<flowstar::Polynomial> *polyVec;
	flowstar::Monomial *mono;
	flowstar::Polynomial *poly;
	flowstar::TaylorModelVec *tmVec;
	flowstar::Matrix *mat;
	std::vector<std::vector<double> > *dVecVec;
	std::vector<flowstar::PolynomialConstraint> *vecConstraints;
	flowstar::ResetMap *resetMap;
	flowstar::Flowpipe *pFlowpipe;
	std::vector<flowstar::Flowpipe> *pVecFlowpipe;
	flowstar::TaylorModel *ptm;
	flowstar::Interval *pint;
	std::vector<std::string> *strVec;
	flowstar::TreeNode *pNode;
	flowstar::UnivariatePolynomial *pUpoly;
	LTI_Term *p_LTI_Term;
	LTV_Term *p_LTV_Term;
	ODE_String *p_ODE_String;

#line 173 "modelParser.tab.h"

};
typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE yylval;


int yyparse (void);


#endif /* !YY_YY_MODELPARSER_TAB_H_INCLUDED  */
