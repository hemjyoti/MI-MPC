\newcommand\classname{base}
\newcommand\lastmodifieddate{2017/07/07}
\newcommand\versionnumber{0.1}

% Are we printing crop marks?
\newif\if@cropmarkson \@cropmarksontrue

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{\classname}[\lastmodifieddate\space\versionnumber]

\setlength{\paperheight}{234truemm}%
\setlength{\paperwidth}{156truemm}%

\DeclareOption{draft}{\PassOptionsToPackage{draft}{graphicx}}
\DeclareOption{a4paper}{\PassOptionsToPackage{a4}{crop}}
\DeclareOption{centre}{\PassOptionsToPackage{center}{}}
\DeclareOption{crop}{\PassOptionsToPackage{cam}{crop}\global\@cropmarksontrue}
\DeclareOption{nocrop}{\PassOptionsToPackage{off}{crop}\global\@cropmarksonfalse}
\DeclareOption{info}{\PassOptionsToPackage{info}{}}
\DeclareOption{noinfo}{\PassOptionsToPackage{noinfo}{crop}}

\ExecuteOptions{a4paper,nocrop,centre,info}

\ProcessOptions

% Load all necessary packages
\RequirePackage{crop,graphicx,amsmath,amsthm,amsfonts,makeidx,color,colortbl}

\def\CNfont{\large\bfseries}
\def\CTfont{\Large\bfseries\boldmath{}}

% Not sure if needed.
%\newcommand\@ptsize{0}

% Set twoside printing
%\@twosidetrue 

% Marginal notes are on the outside edge
%\@mparswitchtrue

% Default font sizes
\def\aboveskip{%
\abovedisplayskip=0pt%
\abovedisplayshortskip=0pt%
}%

\renewcommand\normalsize{%
   \@setfontsize\normalsize{10}{12}
   \abovedisplayskip 12\p@ \@plus2\p@ \@minus5\p@
   \abovedisplayshortskip \z@ \@plus3\p@
   \belowdisplayshortskip 6\p@ \@plus3\p@ \@minus3\p@
   \belowdisplayskip \abovedisplayskip
   \let\@listi\@listI}
\normalsize
\let\@bls\baselineskip

\newcommand\small{%
   \@setfontsize\small{9}{11}%
   \abovedisplayskip 8.5\p@ \@plus3\p@ \@minus4\p@
   \abovedisplayshortskip \z@ \@plus2\p@
   \belowdisplayshortskip 4\p@ \@plus2\p@ \@minus2\p@
   \def\@listi{\leftmargin\leftmargini
               \topsep 4\p@ \@plus2\p@ \@minus2\p@
               \parsep 2\p@ \@plus\p@ \@minus\p@
               \itemsep \parsep}%
   \belowdisplayskip \abovedisplayskip
}

\newcommand\footnotesize{%
   \@setfontsize\footnotesize{8}{9}%
   \abovedisplayskip 6\p@ \@plus2\p@ \@minus4\p@
   \abovedisplayshortskip \z@ \@plus\p@
   \belowdisplayshortskip 3\p@ \@plus\p@ \@minus2\p@
   \def\@listi{\leftmargin\leftmargini
               \topsep 3\p@ \@plus\p@ \@minus\p@
               \parsep 2\p@ \@plus\p@ \@minus\p@
               \itemsep \parsep}%
   \belowdisplayskip \abovedisplayskip
}

\newcommand\scriptsize{\@setfontsize\scriptsize\@viipt\@viiipt}
\newcommand\tiny{\@setfontsize\tiny\@vpt\@vipt}
%\newcommand\little{\@setfontsize\tiny\@ixpt\@ixpt}
\newcommand\large{\@setfontsize\large\@xiipt{14}}
\newcommand\Large{\@setfontsize\Large\@xivpt{18}}
\newcommand\LARGE{\@setfontsize\LARGE\@xviipt{22}}
\newcommand\huge{\@setfontsize\huge\@xxpt{25}}
\newcommand\Huge{\@setfontsize\Huge\@xxvpt{30}}

\DeclareOldFontCommand{\rm}{\normalfont\rmfamily}{\mathrm}
\DeclareOldFontCommand{\sf}{\normalfont\sffamily}{\mathsf}
\DeclareOldFontCommand{\tt}{\normalfont\ttfamily}{\mathtt}
\DeclareOldFontCommand{\bf}{\normalfont\bfseries}{\mathbf}
\DeclareOldFontCommand{\it}{\normalfont\itshape}{\mathit}
\DeclareOldFontCommand{\sl}{\normalfont\slshape}{\@nomath\sl}
\DeclareOldFontCommand{\sc}{\normalfont\scshape}{\@nomath\sc}

% Line spacing
\setlength\lineskip{1\p@}
\setlength\normallineskip{1\p@}
\renewcommand\baselinestretch{}

% Paragraph dimensions and inter-para spacing
\setlength\parskip{0\p@}
\setlength\parindent{1.5em}

% Set inter-para skips
\setlength\smallskipamount{3\p@ \@plus 1\p@ \@minus 1\p@}
\setlength\medskipamount{6\p@ \@plus 2\p@ \@minus 2\p@}
\setlength\bigskipamount{12\p@ \@plus 4\p@ \@minus 4\p@}

% Page break penalties
\@lowpenalty   51
\@medpenalty  151
\@highpenalty 301

% Disallow widows and orphans
\clubpenalty 10000
\widowpenalty 10000

% Disable page breaks before equations, allow pagebreaks after
% equations and discourage widow lines before equations.
\displaywidowpenalty 100
\predisplaypenalty   10000
\postdisplaypenalty  0

% Allow breaking the page in the middle of a paragraph
\interlinepenalty 0

% Disallow breaking the page after a hyphenated line
\brokenpenalty 10000

% Hyphenation; don't split words into less than three characters
\lefthyphenmin=3
\righthyphenmin=3

%
% Set page layout dimensions
%
\setlength\headheight{8\p@}         % height of running head
%\setlength\topmargin{3.2pc}         % head margin
\setlength\topmargin{4.3pc}         % head margin
\addtolength\topmargin{-1in}        % subtract out the 1 inch driver margin

\setlength\topskip{10\p@}           % height of first line of text
\setlength\headsep{36\p@}           % space below running head --
\addtolength\headsep{-\topskip}     %   base to base with first line of text

\setlength\footskip{\z@}            % space above footer line
\setlength\maxdepth{.5\topskip}     % pages can be short or deep by half a line?

\setlength\textwidth{27pc}   %29       % text measure excluding margins

\setlength\textheight{43\baselineskip} % 44 lines on a full page,
\addtolength\textheight{\topskip}      %    including the first
                                       %    line on the page

% Set the margins
% Margin paras are not required. Set lengths to zero.
\setlength\marginparsep{18\p@}
\setlength\marginparpush{6\p@}
\setlength\marginparwidth{40\p@}

\setlength\oddsidemargin{3.5pc}
\addtolength\oddsidemargin{-1.5pc} %%%%%%%%%%by bala
\addtolength\oddsidemargin{-1in}    % subtract out the 1 inch driver margin
\setlength\@tempdima{\paperwidth}
\addtolength\@tempdima{-\textwidth}
\addtolength\@tempdima{-3.5pc}
\setlength\evensidemargin{\@tempdima}
\addtolength\evensidemargin{-1in}
\addtolength\evensidemargin{1.5pc} %%%%%%%%%by bala

\setlength\columnsep{0\p@}          % space between columns for double-column text
\setlength\columnseprule{0\p@}      % width of rule between two columns

% Footnotes
\setlength\footnotesep{6.5\p@\@killglue}     % space between footnotes
% space between text and footnote
\setlength{\skip\footins}{10\p@\@killglue}%

% Float placement parameters

% The total number of floats that can be allowed on a page.
\setcounter{totalnumber}{10}
% The maximum number of floats at the top and bottom of a page.
\setcounter{topnumber}{5}
\setcounter{bottomnumber}{5}
% The maximum part of the top or bottom of a text page that can be
% occupied by floats. This is set so that at least four lines of text 
% fit on the page.
\renewcommand\topfraction{.921}
\renewcommand\bottomfraction{.921}
% The minimum amount of a text page that must be occupied by text.
% This should accomodate four lines of text.
\renewcommand\textfraction{.079}
% The minimum amount of a float page that must be occupied by floats.
\renewcommand\floatpagefraction{.887}

% The same parameters repeated for double column output
\renewcommand\dbltopfraction{.88}
\renewcommand\dblfloatpagefraction{.88}

% Space between floats
\setlength\floatsep    {12\p@ \@plus 2\p@ \@minus 2\p@}
% Space between floats and text
\setlength\textfloatsep{20\p@ \@plus 2\p@ \@minus 4\p@}
% Space above and below an inline figure
\setlength\intextsep   {18\p@ \@plus 2\p@ \@minus 2\p@}

% For double column floats
\setlength\dblfloatsep    {12\p@ \@plus 2\p@ \@minus 2\p@}
\setlength\dbltextfloatsep{20\p@ \@plus 2\p@ \@minus 4\p@}

% Space left at top, bottom and inbetween floats on a float page.
\setlength\@fptop{0\p@}         % no space above float page figures
\setlength\@fpsep{12\p@ \@plus 2fil}
\setlength\@fpbot{0\p@ \@plus 1fil}

% The same for double column
\setlength\@dblfptop{0\p@}
\setlength\@dblfpsep{12\p@ \@plus 1fil}
\setlength\@dblfpbot{0\p@ \@plus 2fil}

% Page styles
%\sodef\track{}{.15em}{0.333em}{0pt}

  \def\ps@headings{%
      \let\@oddfoot\@empty\let\@evenfoot\@empty
      \def\@evenhead{\small{\rlap{\thepage}}\hfill{\MakeUppercase\leftmark}\hfill}%
      \def\@oddhead{\small\hfill{\MakeUppercase\rightmark}\hfill{\llap{\thepage}}}%
      \let\@mkboth\markboth
    \def\chaptermark##1{\markboth{##1}{##1}}%
    \def\sectionmark##1{%
      \markright {{%
        \ifnum \c@secnumdepth >\z@
%          \thesection\ignorespaces\enskip\ignorespaces%
        \fi
        ##1}}}}

\def\ps@plain{%
\let\@oddfoot\relax \let\@evenfoot\relax
%  \def\@oddfoot{\hfill{{\fontsize{10}{12}\selectfont\thepage}}}%
  \let\@evenfoot\@oddfoot%
  \let\@oddhead\relax \let\@evenhead\relax}%
                                                                    
%          
% Sectional units
%

% Lowest level heading that takes a number by default
\setcounter{secnumdepth}{2}
    

% Counters

\newcounter{part}
\newcounter{chapter}
\newcounter{section}[chapter]
\newcounter{subsection}[section]
\newcounter{subsubsection}[subsection]
\newcounter{paragraph}[subsubsection]
\newcounter{subparagraph}[paragraph]
\newcounter{figure}[chapter]
\newcounter{table}[chapter]


% Form of the numbers
\newcommand\thepage{\arabic{page}}
\renewcommand\thepart{\Roman{part}}
\renewcommand\thechapter{\arabic{chapter}}
\renewcommand\thesection{\thechapter.\arabic{section}}
\renewcommand\thesubsection{\thesection.\arabic{subsection}}
\renewcommand\thesubsubsection{{\alph{subsubsection}}}
\renewcommand\theparagraph{\thesubsubsection.\arabic{paragraph}}
\renewcommand\thesubparagraph{\theparagraph.\arabic{subparagraph}}
\renewcommand\theequation{\thechapter.\arabic{equation}}
\newcommand\@chapapp{\chaptername}

% Form of the words
\newcommand\contentsname{Contents}
\newcommand\listfigurename{List of Figures}
\newcommand\listtablename{List of Tables}
\newcommand\partname{Part}
\newcommand\chaptername{Chapter}
\newcommand\appendixname{Application}
\newcommand\abstractname{Abstract}
\newcommand\refname{References}
\newcommand\bibname{Bibliography}
\newcommand\indexname{Index}
\newcommand\figurename{Figure}
\newcommand\tablename{Table}

% Clearemptydoublepage should really clear the running heads too
\newcommand{\clearemptydoublepage}{\newpage{\pagestyle{empty}\cleardoublepage}}

% Frontmatter, mainmatter and backmatter

\newif\if@mainmatter \@mainmattertrue

\newcommand\frontmatter{%
  \clearpage
  \@mainmatterfalse
  \pagenumbering{roman}}

\newcommand\mainmatter{%
  \clearpage
  \@mainmattertrue
  \pagenumbering{arabic}}

\newcommand\backmatter{%
  \clearpage
  \@mainmatterfalse}

\def\chapter{\clearpage\thispagestyle{plain}\global\@topnum\z@
  \secdef\@chapter\@schapter}

\def\@chapter[#1]#2{%
  \ifnum \c@secnumdepth >\m@ne
    \refstepcounter{chapter}
    \typeout{^^J\@chapapp\space\arabic{chapter}:}
    \addcontentsline{toc}{chapter}{\protect\numberline{\thechapter}#2}
  \else
    \addcontentsline{toc}{chapter}{#2}
  \fi
  \chaptermark{#1}
  \@makechapterhead{#2}
  \@afterindentfalse
  \@afterheading}
        
\def\@schapter#1{%
  \chaptermark{#1}
  \thispagestyle{plain}
  \addcontentsline{toc}{chapter}{{\thechapter}#1}
  \@makeschapterhead{#1}
  \@afterindentfalse
  \@afterheading}

\def\@makechapterhead#1{% 
  \begingroup
    \parindent\z@
    \vspace*{17\p@}%
    \hbox to \hsize{\hss\fontsize{14}{16}\CNfont Chapter~\thechapter\hss}\par%
        \removelastskip\nointerlineskip\vskip17.5pt\nointerlineskip%
    {\fontsize{12}{14}\selectfont\centering\CTfont{#1}\par}%
\removelastskip\nointerlineskip\vspace{23pt}\nointerlineskip%
  \endgroup
  \@afterindentfalse
  \@afterheading}

%%%%Appendix Chapter Title
\def\@makeappchapterhead#1{% 
  \begingroup
    \parindent\z@
    \vspace*{17\p@}%
    \hbox to \hsize{\hss\fontsize{14}{16}\selectfont Appendix~\thechapter\hss}\par%
        \removelastskip\nointerlineskip\vskip17.5pt\nointerlineskip%
    {\fontsize{12}{14}\selectfont\centering\uppercase{#1}\par}%
\removelastskip\nointerlineskip\vspace{23pt}\nointerlineskip%
  \endgroup
  \@afterindentfalse
  \@afterheading}

  
  \def\@makeschapterhead#1{%
  \begingroup
    \parindent\z@
    \vspace*{15\p@}%
    {\centering\fontsize{12.5}{14}\selectfont\uppercase{#1}\par}
    \removelastskip\nointerlineskip\vspace*{2.4pc}\nointerlineskip%
  \endgroup
  \@afterindentfalse
  \@afterheading}

% Section macros

% \@sect redefined to write the long entry to the TOC
\def\@sect#1#2#3#4#5#6[#7]#8{%
  \ifnum #2>\c@secnumdepth
    \let\@svsec\@empty
  \else
    \refstepcounter{#1}%
    \protected@edef\@svsec{\@seccntformat{#1}\relax}%
  \fi
  \@tempskipa #5\relax
  \ifdim \@tempskipa>\z@
    \ifnum#2=1
%    \vspace*{-5\p@}
    \fi
    \begingroup
      #6{%
        \@hangfrom{\hskip #3\relax\@svsec}%
          \interlinepenalty \@M #8 \@@par}%
    \endgroup
    \csname #1mark\endcsname{#7}%
    \addcontentsline{toc}{#1}{%
      \ifnum #2>\c@secnumdepth \else
        \protect\numberline{\csname the#1\endcsname}%
      \fi
      #8}%
  \else
    \def\@svsechd{%
      #6{\hskip #3\relax
      \@svsec #8}%
      \csname #1mark\endcsname{#7}%
      \addcontentsline{toc}{#1}{%
        \ifnum #2>\c@secnumdepth \else
          \protect\numberline{\csname the#1\endcsname}%
        \fi
        #8}}%
  \fi
  \@xsect{#5}}


\newdimen\bsecdimen%
\bsecdimen=15pt%%%

\newdimen\asecdimen%
\asecdimen=3pt%

\renewcommand{\@seccntformat}[1]{{\csname the#1\endcsname\ignorespaces\hspace{9pt}\ignorespaces}}
\newcommand\section{\@startsection {section}{1}{\z@}%
                                   {-\bsecdimen \@plus -4\p@}%
                                   {\asecdimen}
{\normalsize\bfseries\mathversion{bold}\raggedright}}

\newdimen\bsubsecdimen% B and C level above spaces are same
\bsubsecdimen=15pt%%

\newdimen\asubsecdimen%
\asubsecdimen=3pt%%

\newcommand\subsection{\@startsection{subsection}{2}{\z@}%
                                     {-\bsubsecdimen \@plus -3\p@}%
                                     {\asubsecdimen}%
{\normalsize\itshape\raggedright}}

\newcommand\subsubsection{\@startsection{subsection}{3}{\z@}%
                                     {-\bsubsecdimen \@plus -3\p@}%
                                     {\asubsecdimen}%
{\normalsize\sf\itshape\raggedright}}

\newcommand\paragraph{\@startsection{paragraph}{4}{\z@}%
                                     {-6\p@ \@plus -3\p@}%
                                     {-1em}%
{\fontsize{10.5}{12}\selectfont\raggedright\itshape}}

\newlength\abovecaptionskip
\newlength\belowcaptionskip
\setlength\abovecaptionskip{6\p@}
\setlength\belowcaptionskip{5\p@}

\newcommand{\lastline}[1]{\break \bgroup\hbox to \hsize{\hfill
{#1}\hfill}\egroup}%

\long\def\@makecaption#1#2{\vspace{\abovecaptionskip}%
\normalsize%
  \setbox\@tempboxa=\hbox{#1\hspace{1em}#2}%
  \ifdim\wd\@tempboxa>\hsize
\centering  {\sc #1.}\ignorespaces\enskip\ignorespaces#2%
  \else
    \hbox to \hsize{\hss{\sc #1.}\ignorespaces\enskip\ignorespaces#2\hss}
  \fi}

\long\def\@tablecaption#1#2{\fontsize{9.5}{10}\selectfont%
{\bfseries #1}\hspace{1em}{#2\strut}\par%
  \vspace{\belowcaptionskip}}%

\newcommand{\notename}{Notes{\em:}}

% Top rule
\def\toprule{\noalign{\vskip3pt}
\noalign{\ifnum0=`}\fi\hrule 
  \@height 1\p@ % <- rule height
  \hrule \@height 4pt \@width 0pt 
  \futurelet\@tempa\@xhline
  \noalign{\vskip-.7pt}
  }
% Bottom rule
\def\botrule{\noalign{\ifnum0=`}\fi \hrule \@height 3.2pt \@width 0pt 
  \hrule \@height 1\p@ % <- rule height 
  \futurelet\@tempa\@xhline
  \noalign{\vskip1pt}}
% Middle rule
\def\midrule{\noalign{\ifnum0=`}\fi \hrule \@height 2.25pt \@width 0pt 
  \hrule \@height 0.5pt % <- rule height 
  \hrule \@height 5pt \@width 0pt \futurelet \@tempa\@xhline}
% Bridge rule
\def\hrulefill{\leavevmode\leaders\hrule height .5pt\hfill\kern\z@}

\def\hrulefill{\leavevmode\leaders\hrule height .50pt\hfill\kern\z@}

\def\thefigure{\thechapter.\@arabic\c@figure}
\def\fps@figure{tbp}
\def\ftype@figure{1}
\def\ext@figure{lof}
\def\fnum@figure{\figurename~\thefigure}
\def\figure{\@float{figure}}
\def\endfigure{\end@float}
\@namedef{figure*}{\@dblfloat{figure}}
\@namedef{endfigure*}{\end@dblfloat}
\def\thetable{\thechapter.\@arabic\c@table}
\def\fps@table{tbp}
\def\ftype@table{2}
\def\ext@table{lot}
\def\fnum@table{Table~\thetable}
\def\table{\let\@makecaption\@tablecaption\@float{table}}
\let\endtable\end@float
\@namedef{table*}{\let\@makecaption\@tablecaption\@dblfloat{table}}
\@namedef{endtable*}{\end@dblfloat}

\newif\if@rotate \@rotatefalse
\newif\if@rotatecenter \@rotatecenterfalse
\def\rotatecenter{\global\@rotatecentertrue}
\def\rotateendcenter{\global\@rotatecenterfalse}
\def\rotate{\global\@rotatetrue}
\def\endrotate{\global\@rotatefalse}
\newdimen\rotdimen
\def\rotstart#1{\special{ps: gsave currentpoint currentpoint translate
    #1 neg exch neg exch translate}}
\def\rotfinish{\special{ps: currentpoint grestore moveto}}
\def\rotl#1{\rotdimen=\ht#1\advance\rotdimen by \dp#1
    \hbox to \rotdimen{\vbox to\wd#1{\vskip \wd#1
    \rotstart{270 rotate}\box #1\vss}\hss}\rotfinish}
\def\rotr#1{\rotdimen=\ht #1\advance\rotdimen by \dp#1
    \hbox to \rotdimen{\vbox to \wd#1{\vskip \wd#1
    \rotstart{90 rotate}\box #1\vss}\hss}\rotfinish}

\newdimen\tempdime
\newbox\temptbox
\newenvironment{processtable}[3]{\setbox\temptbox=\hbox{\fontsize{9}{11}\selectfont{#2}}%
\tempdime\wd\temptbox\@processtable{#1}{#2}{#3}{\tempdime}}
{\relax}

\newlength{\tabbotskip}%
\setlength{\tabbotskip}{\z@}%
\newenvironment{@processtable}[4]{%
\if@rotate
\setbox4=\vbox to \textwidth{\vss\hbox to \textheight{\hss%
\begin{minipage}{#4}%
\fontsize{9}{10}\selectfont
\caption{#1}{#2}%
\vskip4pt\noindent
\parbox{#4}{\fontsize{8}{10}\selectfont #3\par}%
\end{minipage}}\vss}%
\else
\setbox4=\hbox to \textwidth{\vbox{\begin{center}\begin{minipage}[t]{#4}%
\fontsize{9}{10}\selectfont
\caption{#1}{#2}%
\vskip4pt\noindent
\parbox{#4}{\fontsize{8}{10}\selectfont #3\vphantom{y}\par}%
\if!#3!\setlength{\tabbotskip}{-19\p@}\else\setlength{\tabbotskip}{-9.3\p@}\fi%
\vspace*{\tabbotskip}%
\end{minipage}\end{center}}}%
\fi
\if@rotate\rotl{4}\else\box4\fi}
{\relax}

% ******************************
% List numbering and lettering *
% ******************************
\def\labelenumi{{\rm\arabic{enumi}.}}
\def\theenumi{\arabic{enumi}}
\def\labelenumii{{\rm \roman{enumii}.}}
\def\theenumii{a\alph{enumii}}
\def\p@enumii{\theenumi}
\def\labelenumiii{({\rm\roman{enumiii}})}
\def\theenumiii{\roman{enumiii}}
\def\p@enumiii{\theenumi(\theenumii)}
\def\labelenumiv{({\rm\Alph{enumiv}})}
\def\theenumiv{\Alph{enumiv}}
\def\p@enumiv{\p@enumiii\theenumiii}
\def\labelitemi{$\bullet$}
\def\labelitemii{$\bullet$}
\def\labelitemiii{$\bullet$}
\def\labelitemiv{$\bullet$}
      
\setlength\leftmargini   {15pt}%
\leftmargin  \leftmargini
\setlength\leftmarginii  {\z@}
\setlength\leftmarginiii {\z@}
\setlength\leftmarginiv  {\z@}
\setlength\leftmarginv   {\z@}
\setlength\leftmarginvi  {\z@}

\setlength\partopsep{2\p@ \@plus 1\p@ \@minus 1\p@}
\setlength  \labelsep  {.5em}
\setlength  \labelwidth{\leftmargini}
\addtolength\labelwidth{-\labelsep}

\def\@listi{\leftmargin\leftmargini
            %\parsep 4\p@ \@plus2\p@ \@minus\p@
            \topsep 6\p@ \@plus2\p@ \@minus4\p@
            %\itemsep4\p@ \@plus2\p@ \@minus\p@
            }
\let\@listI\@listi
\@listi
\def\@listii {\leftmargin\leftmarginii
              \labelwidth\leftmarginii
              \advance\labelwidth-\labelsep
              \topsep   \z@% 4\p@ \@plus2\p@ \@minus\p@
              \parsep    2\p@ \@plus\p@  \@minus\p@
              \itemsep   \parsep}
\def\@listiii{\leftmargin\leftmarginiii
              \labelwidth\leftmarginiii
              \advance\labelwidth-\labelsep
              \topsep    2\p@ \@plus\p@\@minus\p@
              \parsep    \z@
              \partopsep \p@ \@plus\z@ \@minus\p@
              \itemsep   \topsep}
\def\@listiv {\leftmargin\leftmarginiv
              \labelwidth\leftmarginiv
              \advance\labelwidth-\labelsep}
\def\@listv  {\leftmargin\leftmarginv
              \labelwidth\leftmarginv
              \advance\labelwidth-\labelsep}
\def\@listvi {\leftmargin\leftmarginvi
              \labelwidth\leftmarginvi
              \advance\labelwidth-\labelsep}

\def\enumargs{%
   \partopsep     \z@
   \itemsep       \z@
   \parsep        \z@
   \labelsep      0.5em
   \rightmargin   \z@
   \listparindent \parindent
   \itemindent    \z@}

\def\enumerate{%
    \@ifnextchar[{\@numerate}{\@numerate[0.]}}

\def\@numerate[#1]{%
     \ifnum \@enumdepth >3 \@toodeep\else
     \advance\@enumdepth \@ne
     \edef\@enumctr{enum\romannumeral\the\@enumdepth}
     \list{\csname label\@enumctr\endcsname}{%
       \enumargs
       \setlength{\leftmargin}{\csname leftmargin\romannumeral\the\@enumdepth\endcsname}
       \usecounter{\@enumctr}
       \settowidth\labelwidth{#1}
       \addtolength{\leftmargin}{\labelwidth}
       \addtolength{\leftmargin}{\labelsep}
       \def\makelabel##1{\hss\llap{##1}}}%
     \fi
   }
\let\endenumerate\endlist


% Changes to the list parameters for itemize
\def\itemargs{%
   \leftmargini   15\p@
   \partopsep     \z@
   \itemsep       \z@
   \parsep        \z@
   \labelsep      0.5em
   \rightmargin   \z@
   \listparindent \parindent
   \itemindent    \z@}

\def\itemize{%
   \@ifnextchar[{\@itemize}{\@itemize[$\bullet$]}}

\def\@itemize[#1]{%
     \ifnum \@itemdepth >3 \@toodeep\else
     \advance\@itemdepth \@ne
     \edef\@itemctr{item\romannumeral\the\@itemdepth}
     \list{\csname label\@itemctr\endcsname}{%
       \itemargs
       \setlength{\leftmargin}{\csname leftmargin\romannumeral\the\@itemdepth\endcsname}
       \settowidth\labelwidth{#1}
       \addtolength{\leftmargin}{\labelwidth}
       \addtolength{\leftmargin}{\labelsep}
       \def\makelabel##1{\hss \llap{##1}}}%
     \fi
   }
\let\enditemize\endlist

\def\unargs{%
   \partopsep     \z@
   \itemsep       \z@
   \labelwidth    \z@
   \parsep        \z@
   \labelsep      \z@
   \rightmargin   \z@
   \listparindent \parindent
   \leftmargin 12pt
   \itemindent -6pt}

\newenvironment{unlist}{%
\begin{list}{}{%
  \unargs%
  }}{\end{list}}

\newenvironment{description}
               {\list{}{\labelwidth\z@ \itemindent-\leftmargin
                        \let\makelabel\descriptionlabel}}
               {\endlist}
\newcommand*\descriptionlabel[1]{\hspace\labelsep
                                \normalfont\bfseries #1}
\newenvironment{verse}
               {\let\\\@centercr
                \list{}{\itemsep      \z@
                        \itemindent   -1.5em%
                        \listparindent\itemindent
                        \rightmargin  \leftmargin
                        \advance\leftmargin 1.5em}%
                \item\relax}
               {\endlist}
\newenvironment{quotation}
               {\list{}{\listparindent 1.5em%
                        \itemindent    \listparindent
                        \rightmargin   \leftmargin
                        \parsep        \z@ \@plus\p@}%
                \item\relax}
               {\endlist}
\newenvironment{quote}
               {\list{}{\leftmargin18pt}%
                \small
                \item\relax}
               {\endlist}

\skip\@mpfootins = \skip\footins
\fboxsep=6\p@
\fboxrule=1\p@

\def\thenumbibliography#1{\chapter*{\refname}\small\list
 {[\arabic{enumi}]}{\labelsep 0.5em%
 \hyphenpenalty=50%
 \settowidth\labelwidth{[#1]}\leftmargin\labelwidth
 \advance\leftmargin\labelsep
 \usecounter{enumi}
  \global\itemsep-.1pc
}
 \def\newblock{\hskip .14em plus .38em minus -.1em}
 \sloppy
 \sfcode`\.=1000\relax}
\let\endthenumbibliography=\endlist

%%
\newenvironment{theunbibliography}[1]{%
 \chapter*{\refname}
\markboth{\refname}{\refname}
 \normalsize\list{}{%
  \usecounter{enumi}%
  \leftmargin 1em\itemindent -1em\parsep \z@
%    \itemsep3pt 
 }}
{\endlist}


\RequirePackage{multicol}

\newif\if@restonecol
\def\theindex{\@mainmatterfalse
  \addcontentsline{toc}{chapter}{Index}
  \columnseprule \z@
  \columnsep 3pc\begin{multicols}{2}[\chapter*{\indexname}] 
                                %Subject or Author here
  \markboth{\indexname}{\indexname}%
  \parskip\z@ plus .3\p@\relax\let\item\idxitem\fontsize{8}{9.5}\selectfont}
\def\idxitem{\par\noindent\raggedright\hangindent 3em}
\def\subitem{\par\noindent\raggedright\hangindent 4em\hspace*{1em}}
\def\subsubitem{\par\noindent\raggedright\hangindent 5em\hspace*{2em}}
\def\endtheindex{\end{multicols}}
\def\indexspace{\par \vskip 12\p@ plus 5\p@ minus 3\p@\relax}
     
% ***********
% Footnotes *
% ***********

\renewcommand\footnoterule{\hbox to 6pc{\hrulefill}\par\removelastskip\nointerlineskip\vskip3.5pt\nointerlineskip}%

\newcommand\@makefntext[1]{%
    \fontsize{8.5}{9}\selectfont%\global\baselineskip=9pt%
    \parindent10pt%
    \noindent\ignorespaces\hspace*{10pt}\ignorespaces
    \hbox{\@makefnmark}#1}
\def\@makefnmark{\hbox{\smash{\@textsuperscript{\normalfont\@thefnmark}}}}

%\@addtoreset{footnote}{page}
    
\newcommand{\cfootnote}[1]{\begingroup\let\@makefntext\@makecnfntext\footnote{#1}\endgroup}

\newcommand\@makecnfntext[1]{%
    \footnotesize
    \parindent 1em
    \noindent\centering
    \hbox{\smash{\@makefnmark}\enskip}#1}

%\def\@fnsymbol#1{\ensuremath{\ifcase#1\or \dagger\or \ddagger\or \mathsection\or \undefined \or \undefined \or
%   \undefined \else\@ctrerr\fi}}
   
%\renewcommand\thefootnote{\fnsymbol{footnote}}%

%Math parameters

\setlength{\jot}{5\p@} 
\mathchardef\@m=1500   % adapted value

\def\frenchspacing{\sfcode`\.\@m \sfcode`\?\@m \sfcode`\!\@m
  \sfcode`\:\@m \sfcode`\;\@m \sfcode`\,\@m}

\def\@mathmargin{1.5pc}
\vbadness=9999
\tolerance=9999
\doublehyphendemerits=10000
\doublehyphendemerits 640000   % corresponds to badness 800
\finalhyphendemerits  1000000  % corresponds to badness 1000

% Table and array parameters
\setlength\arraycolsep{.5em}
\setlength\tabcolsep{.5em}
\setlength\arrayrulewidth{.25pt}
\setlength\doublerulesep{2.5pt}
\renewcommand\arraystretch{1}

\def\@dottedtocline#1#2#3#4#5{%
  \renewcommand\@dotsep{1000}
  \ifnum #1>\c@tocdepth \else
    \vskip \z@ \@plus.2\p@
    {\leftskip #2\relax \rightskip \@tocrmarg \parfillskip -\rightskip
     \parindent #2\relax\@afterindenttrue
     \interlinepenalty\@M
     \leavevmode
     \@tempdima #3\relax
     \advance\leftskip \@tempdima \null\nobreak\hskip -\leftskip
     {#4}\nobreak
     \leaders\hbox{$\m@th \mkern \@dotsep mu\hbox{.}\mkern \@dotsep
        mu$}\hfill
     \nobreak
     \hb@xt@\@pnumwidth{\hfil\normalcolor\normalsize #5}%
     \par}%
  \fi}


\def\@authordottedtocline#1#2#3#4#5{%
    \renewcommand\@dotsep{1000}
  \ifnum #1>\c@tocdepth \else
    \vskip 0\p@ \@plus.2\p@
    {\leftskip #2\relax \rightskip \@tocrmarg \parfillskip -\rightskip
     \parindent #2\relax\@afterindenttrue
     \interlinepenalty\@M
     \leavevmode
     \@tempdima #3\relax
     \advance\leftskip \@tempdima \null\nobreak\hskip -\leftskip
     {\it #4}\nobreak
     \leaders\hbox{$\m@th \mkern \@dotsep mu\hbox{.}\mkern \@dotsep
        mu$}\hfill
  \nobreak
     \hb@xt@\@pnumwidth{\hfil\normalcolor\normalsize}%
     \par}%
\vspace{6pt}
  \fi}


\def\@chapterdottedtocline#1#2#3#4#5{%
  \begingroup
\def\numberline##1{\hb@xt@\@tempdima{{##1}\hfil}}
  \renewcommand\@dotsep{1000}
  \ifnum #1>\c@tocdepth \else
    \vskip 5pt \@plus 1pt\goodbreak
    {\leftskip #2\relax   \rightskip \@tocrmarg \parfillskip -\rightskip
     \parindent #2\relax\@afterindenttrue
     \interlinepenalty\@M
     \leavevmode
     \@tempdima #3\relax
     \advance\leftskip \@tempdima \null\nobreak\hskip -\leftskip
     {\bfseries #4}\nobreak
     \leaders\hbox{$\m@th         \mkern \@dotsep mu\hbox{.}\mkern \@dotsep
        mu$}\hfill
      \nobreak
     \hb@xt@\@pnumwidth{\hfil\normalcolor\normalsize { #5}}%
     \par}%
  \endgroup
  \fi}

\def\@appendixdottedtocline#1#2#3#4#5{%
  \begingroup
  \renewcommand\@dotsep{1000}
  \ifnum #1>\c@tocdepth \else
    \vskip 9pt \@plus 1pt\goodbreak
    {\leftskip #2\relax   \rightskip \@tocrmarg \parfillskip -\rightskip
     \parindent #2\relax\@afterindenttrue
     \interlinepenalty\@M
     \leavevmode
     \@tempdima #3\relax
     \null\nobreak\hskip -\leftskip
     {\bfseries #4}\nobreak
     \leaders\hbox{$\m@th         \mkern \@dotsep mu\hbox{.}\mkern \@dotsep
        mu$}\hfill
     \nobreak
     \hb@xt@\@pnumwidth{\hfil\normalcolor\normalsize {\bfseries #5}}%
     \par}%
  \endgroup
  \fi}

\def\@partdottedtocline#1#2#3#4#5{%
  \begingroup
  \def\numberline##1{\hbox{{Part ##1\quad}}}
  \renewcommand\@dotsep{1000}
  \ifnum #1>\c@tocdepth \else
    \vskip 16pt \@plus 1pt\goodbreak
    {\leftskip #2\relax   \rightskip \@tocrmarg \parfillskip -\rightskip
     \parindent #2\relax\@afterindenttrue
     \interlinepenalty\@M
     \leavevmode
     \@tempdima #3\relax
     \null\nobreak\hskip -\leftskip
     {\fontsize{11}{13}\selectfont\bfseries #4}\nobreak
     \leaders\hbox{$\m@th         \mkern \@dotsep mu\hbox{.}\mkern \@dotsep
        mu$}\hfill
     \nobreak
     \hb@xt@\@pnumwidth{\hfil\normalcolor\normalsize {\fontsize{11}{13}\selectfont\bfseries #5}}%
     \par}%
  \endgroup
  \fi}

\newcommand\@pnumwidth{1.5pc}
\newcommand\@tocrmarg{2.55em plus 1fil}
\newcommand\@dotsep{1}             %%% <-- very large value so we don't get any dots
\newcommand\tableofcontents{%
    \if@twocolumn
      \@restonecoltrue\onecolumn
    \else
      \@restonecolfalse
    \fi

    \chapter*{Contents}
\markboth{Contents}{Contents}
    \chaptermark{Contents}       %% <-- check the running heads in all \chapter*
    \@starttoc{toc}%                    %%      Code manually with \markboth{}{}
    \if@restonecol\twocolumn\fi
    }

\setcounter{tocdepth}{3}

\newcommand*\l@book[1]{{\vspace{\bigskipamount}\vspace{\medskipamount}\noindent{\large\bfseries #1}}}
\newcommand*\l@part{\@partdottedtocline{0}{0em}{60pt}}
\newcommand*\l@prelims{\@dottedtocline{0}{0em}{20pt}}
\newcommand*\l@chapter{\@chapterdottedtocline{1}{0pt}{21pt}}
\newcommand*\l@section{\@dottedtocline{2}{21pt}{27pt}}
\newcommand*\l@author{\@authordottedtocline{2}{15pt}{27pt}}
\newcommand*\l@subsection{\@dottedtocline{3}{47.5pt}{29.9pt}}
\newcommand*\l@subsubsection[2]{}
\newcommand*\l@paragraph[2]{}
\newcommand*\l@subparagraph[2]{}
\newcommand*\l@appendix{\@appendixdottedtocline{2}{0em}{17pt}}
\newcommand*\l@appsection{\@dottedtocline{2}{17pt}{23pt}}
\newcommand*\l@endmatter{\@chapterdottedtocline{2}{0em}{0em}}

\flushbottom
\frenchspacing
\ps@headings
\onecolumn

\graphicspath{{g:/artwork/oup/Jork/}}

\hyphenation{Figure Figures Table Tables Section Sections}

% Reset eqnarray to avoid to avoid \arraycolsep between 
% columns. Take from a post on comp.text.tex 
% by Fred Bartlett <fbartlet@optonline.net> 
\newif\if@alignpoint
\def\eqnarray{%
  \stepcounter{equation}%
  \def\@currentlabel{\p@equation\theequation}%
  \global\@eqnswtrue
  \m@th
  \global\@eqcnt\z@
  \tabskip\@centering
  \let\\\@eqncr
  $$\everycr{}\halign to\displaywidth\bgroup
    \hskip\@centering$\displaystyle\tabskip\z@skip{##}$\@eqnsel
  &\global\@eqcnt\@ne\hfil\setbox\z@\hbox{$\displaystyle{{}##{}}$}%
    \global\ifdim\wd\z@>\z@\@alignpointtrue\else\@alignpointfalse\fi
    \box\z@\hfil
  &\global\@eqcnt\tw@$\displaystyle{\if@alignpoint\else{}\fi##}$\hfil
    \tabskip\@centering
  &\global\@eqcnt\thr@@ \hb@xt@\z@\bgroup\hss##\egroup
         \tabskip\z@skip
  \cr}

% Local modifications

% Load the correct font sizes

\DeclareFontShape{OT1}{cmr}{m}{n}
   {  <5> <6> <7> <8> <9> <10> <12> gen * cmr
      <11> cmr10
      <15><14.4><14><13>  cmr12
      <18><20.74><24><24.88><26.10><32>cmr17}{}

\DeclareFontShape{OT1}{cmr}{bx}{n}
   {  <5> <6> <7> <8> <9> gen * cmbx
      <10><11> cmbx10
      <12><14><17.28><20.74><24.88><33.5><32.5>cmbx12}{}

\def\th@definition{%
  \thm@headsep 1em minus\p@\relax
%%  \let\thm@indent\noindent % no indent
\thm@headfont{\bfseries}% heading font is bold
%%  \thm@notefont{}% same as heading font
\thm@headpunct{}% no period after heading
%%  \let\thm@swap\@gobble
\thm@preskip\bigskipamount
%%  \thm@postskip\theorempreskipamount
  \itshape % body font
}

%%%Appendix%%
\newcounter{appcounter}%
\setcounter{appcounter}{0}
%\refstepcounter{appcounter}%
\renewcommand{\theappcounter}{\Alph{appcounter}}%

\def\appendix{\par
\let\@makechapterhead=\@makeappchapterhead%
\refstepcounter{appcounter}%
  \renewcommand{\thechapter}{\Alph{chapter}}
  \renewcommand{\thesection}{\thechapter.\arabic{section}}
  \renewcommand{\@seccntformat}[1]{{\csname
  the##1\endcsname}\quad}
\def\thetable{\thechapter\Alph{section}.\@arabic\c@table}
        \setcounter{section}{0}
        \setcounter{table}{0}
        \setcounter{subsection}{0}
%        \appendtrue
\def\@chapapp{App.\ }
\def\chaptername{Appendix}}

\newtheoremstyle{common}
    {6pt plus 5\p@ minus 2\p@}% above space (default)
    {6pt plus 5\p@ minus 2\p@}% below space
    {\itshape}% body
    {0em}% indent
    {\bfseries}% head
    {}% punct
    {.5em}% space
    {}% custom
\theoremstyle{common}

%%%With Chapter Number
\newtheorem{theorem}{Theorem}[chapter]%
\newtheorem{fp}{Fundamental Problems}[chapter]%
\newtheorem{corollary}[theorem]{Corollary}%
\newtheorem{example}[theorem]{Example}%
\newtheorem{lemma}[theorem]{Lemma}%
\newtheorem{definition}[theorem]{Definition}%
\newtheorem{proposition}[theorem]{Proposition}%
%%%With Section Number%%
\newtheorem{assumption}{Assumption}[section]%
\newtheorem{condition}{Condition}[section]%
\newtheorem{idea}{Idea}[section]%
\newtheorem{remark}{Remark}[section]%
\newtheorem{result}{Result}[section]%
\newtheorem{nota}{Notation convention}[section]%

%%%Proof%%
%%For Proof definition inside the amsthm.sty%%%

%%%Boxed Text%%%
\newenvironment{QBoxtext}[1]{\begingroup\parindent\z@%
\removelastskip\nointerlineskip\vskip9pt\nointerlineskip%
\fboxrule=0.5pt%
\fboxsep=6pt%
\hbox to \textwidth{\hss\fbox{\parbox{15.5pc}{\parindent\z@%
\centering#1}}\hss}}{\removelastskip\nointerlineskip\vskip9pt\nointerlineskip%
\endgroup}%

\newenvironment{ABoxtext}[1]{\begingroup\parindent\z@%
\removelastskip\nointerlineskip\vskip15pt\nointerlineskip%
\fboxrule=0.5pt%
\fboxsep=6pt%
\hbox to \textwidth{\hss\fbox{\parbox{10.5pc}{\parindent\z@%
\centering#1}}\hss}}{\removelastskip\nointerlineskip\vskip9pt\nointerlineskip%
\endgroup}%

\newenvironment{multilist}[1]{\begingroup%
\begin{center}%
\begin{tabular}[t]{@{}p{10pc}@{\hspace*{1em}}p{10pc}}%
#1}{\end{tabular}%
\end{center}\endgroup}%

%%%Exercises%%%
\newcommand{\exername}{Exercises}%
\newenvironment{Exercises}{\begingroup%
\newcounter{exercise}%
\setcounter{exercise}{0}%
\@addtoreset{exercise}{chapter}%
\newenvironment{Exer}{\begin{list}{%
\textbf{Exercise~\thechapter.\arabic{exercise}}\enskip}{\usecounter{exercise}%
\setlength{\labelsep}{\z@}%
\setlength{\leftmargin}{\z@}%
\setlength{\labelwidth}{\z@}%
\setlength{\itemsep}{5\p@}%{3\p@}
\setlength{\listparindent}{1.5em}}}%
{\end{list}}%
\section{\exername}%
\def\Hint{\textbf{Hint:}\enskip}%
\begin{Exer}
\item[]
}%
{\end{Exer}\endgroup}%


\language=0%%

\let\cal\mathcal

\makeindex

% Author queries
\newcommand{\query}[2][0pt]{%
  \marginpar{\vspace*{#1}%
    \fbox{\parbox{6pc}{%
     \raggedright\small
        AQ: #2}}}}

\def\uns{~}%
\def\boxindent{\hspace*{9pt}}
%%defined by diacritech
\DeclareFixedFont{\boxsubsectionfont}{OT1}{gly}{b}{n}{8.5}%used for cox
%\font\boxeqnofont=cob at 8.5pt

\newenvironment{mibox}
               {\def\@eqnnum{{(\theequation)
               }}
               \list{}{\rightmargin14pt
               \leftmargin15pt}%
                \item \raggedright\fontsize{8.5}{11}\selectfont\sf\relax}
               {\endlist
               \def\@eqnnum{{\normalfont\normalcolor (\theequation)}}}


\def\brule{\noindent\hspace*{-13.97pt}\rule{113.26mm}{2pt}}

\def\boxhead#1{
{\fontsize{9}{12}\selectfont\sf\bfseries \MakeUppercase{#1}}\par%
\vspace*{-2mm}
\noindent\rule{108mm}{0.5pt}\par%
\vspace*{2mm}\vspace*{-5pt}
}

\def\boxsubhead#1{{\vspace*{6pt}\par\noindent\boxsubsectionfont #1\par}\vspace*{3pt}}

\long\def\boxfigcap#1{\vskip7pt%
\setbox\@tempboxa\hbox{%
\hspace*{5.4mm}\parbox{107.5mm}{{\fnum@figure}\par\vspace*{4pt}\hrule width108mm\vspace*{3pt}
{\raggedright\fontsize{8.5}{11}\selectfont\sf #1\par}}}%
\ifdim \wd\@tempboxa >\hsize \par\unhbox\@tempboxa%
\else \hbox to 110mm{\box\@tempboxa}\fi
\vspace*{12pt}}

\definecolor{csblack}{cmyk}{0,0,0,.2}
\newenvironment{boxtext}[1]{\begingroup\vspace*{6.2pt}%
\raggedright
\begin{shaded}%
\vspace*{-4.5pt}%
%
%\raggedright
{#1}           %
\vspace*{-6.5pt}%%
\end{shaded}%
}
{\vspace*{-1pt}\endgroup}

\newcounter{exampled}
\newenvironment{exampled}
    {\stepcounter{exampled}\vskip2pt%
    \noindent\rule{\textwidth}{2pt}\par%
    \noindent\llap{\sf\bfseries Example \thechapter.\theexample\hspace*{7.3pt}}\hspace*{-4pt}%
    \fontsize{9}{12}\selectfont\sf%
    }%
    {\vskip6pt}%

\newenvironment{solution}
    {\noindent\llap{\sf\bfseries\slshape Solution\hspace*{7.9pt}}\hspace*{-3.5pt}%
    \fontsize{9}{12}\selectfont\sf%
    }
    {\par%
    \vspace*{-5pt}%
    \noindent\rule{\textwidth}{0.5pt}\vskip6.3pt}

\setlength{\marginparwidth}{70pt}

\long\def\@ympar#1{%
  \@savemarbox\@marbox{\raggedright #1}%
  \global\setbox\@currbox\copy\@marbox
  \@xympar}

\endinput
