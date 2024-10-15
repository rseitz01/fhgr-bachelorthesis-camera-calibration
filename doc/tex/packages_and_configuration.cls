% !TEX root = documentation.tex
%\documentclass[a4paper,12pt]{IEEEtran}
\documentclass[a4paper,12pt]{scrartcl}


% imports {{{

%\titlespacing*{\section}{0pt}{0.1\baselineskip}{0.2\baselineskip}

% ------------------------
\usepackage[ngerman]{babel}
\usepackage[%showframe,
            %includehead,
            %includefoot,
            top=2.5cm,
            left=2.5cm,
            right=2.5cm,
            bottom=2.0cm,
            headsep=\dimexpr 1.2cm-\baselineskip,
            headheight=\baselineskip,
            footskip=1.2cm,
            ]{geometry}

\usepackage{fontspec}
\usepackage{fontenc}
\defaultfontfeatures{Mapping=tex-text,Scale=MatchLowercase}
\setmainfont{Arial}
\usepackage{float}
%\setmonofont{Lucida Sans Typewriter}

\usepackage[style=ieee]{biblatex}
\addbibresource{references.bib}

% https://tex.stackexchange.com/questions/579848/biblatex-empty-parentheses-for-no-author-after-authors-name
% gets rid of year field for @online
\usepackage{xpatch}
\xpatchbibdriver{online}
  {\printtext[parens]{\usebibmacro{date}}}
  {\iffieldundef{year}{}{\printtext[parens]{\usebibmacro{date}}}}
  {}{}  



%\bibliographystyle{IEEEtranM}

\usepackage{mathtools}  % mathllap
\usepackage{abstract}
\usepackage{hyperref}
%\usepackage{apacite}
\usepackage{tabularx}
%\usepackage{booktabs}   % tabular
\usepackage{longtable}  % table across pages
\usepackage[affil-it]{authblk}
%\usepackage{newtxtext}
%\usepackage{newtxmath}
\setkomafont{disposition}{\bfseries}

\usepackage{ragged2e} % RaggedRight

\usepackage{amsmath}

\usepackage[dvipsnames]{xcolor}     % HTML colors
\usepackage{color}  % colors and colorbox
\definecolor{lightblue}{HTML}{7F7FFF}

% be a bit more relaxed about streching...
% https://tex.stackexchange.com/questions/113805/linebreak-before-texttt
\setlength\emergencystretch{3cm}

\usepackage{graphicx}
\graphicspath{{.}}
\usepackage{float} % https://tex.stackexchange.com/questions/8625/force-figure-placement-in-text

%\renewcommand{\baselinestretch}{1.5}
%\renewcommand{\baselineskip}{1.5}
%\addtolength{\parskip}{1.3mm}

%\usepackage{setspace}
%\onehalfspacing
%\renewcommand{\baselinestretch}{1.5}
%\setstrech{1.3} % 1.3 == one-and-a-half-spacing

%\onehalfspacing

\usepackage{enumitem}   % setlist -> nosep/nolistsep

\usepackage{amsfonts}   % mathbb

% https://tex.stackexchange.com/questions/234777/changing-figure-caption-text-size
% https://tex.stackexchange.com/questions/24599/what-point-pt-font-size-are-large-etc
\usepackage{caption}
\captionsetup[figure]{font=footnotesize,labelfont=footnotesize}
% footnotesize @ 12pt ==> is 10pt


% https://tex.stackexchange.com/questions/37879/how-to-set-the-header-font-size-using-fancyhdr-package
\usepackage{fancyhdr}
\pagestyle{fancy}
\fancyhf{} % clear existing header/footer entries
% https://tex.stackexchange.com/questions/13896/how-to-remove-the-top-horizontal-bar-in-fancyhdr
\renewcommand{\headrulewidth}{0pt} % remove top line
\fancyhead[L]{\fontsize{10pt}{10pt}\selectfont Entwicklung eines automatischen Timing-Messgeräts für Kameras}
\fancyhead[R]{\fontsize{10pt}{10pt}\selectfont Raphael Seitz}
\fancyfoot[R]{\fontsize{10pt}{10pt}\selectfont\thepage}

% https://tex.stackexchange.com/questions/103286/how-to-change-section-subsection-font-size
\usepackage{sectsty}
\sectionfont{\fontsize{14}{14}\selectfont}
\subsectionfont{\fontsize{12}{12}\selectfont}
\subsubsectionfont{\fontsize{12}{12}\selectfont}
\paragraphfont{\fontsize{12}{12}\selectfont}



% }}}

% https://stackoverflow.com/questions/1061112/eliminate-space-before-beginitemize
%\setlist{nolistsep}
\setlist{nosep}         % remove space above itemize
%\setlist{leftmargin=4.5mm}
%\renewcommand\leftmargin{}
%\setlist[itemize]{noitemsep, topsep=1.5pt}


% Between an itemize environment and its preceding text, a length, \parskip, also adds to that space. Enumitem package does not control it. To completely eliminate that space without changing \parskip, you can use
% \setlist{nosep,topsep=-\parskip}



% configuration {{{
% ------------------------

% paragraph indent (left side)
\setlength{\parindent}{0pt}

% hyperlink colors
\hypersetup{
         colorlinks,
         citecolor=black,
         filecolor=black,
         linkcolor=black,
         urlcolor=black,
         pdfborder=0 0 0
}

%\linespread{1.25}

% https://tex.stackexchange.com/questions/49188/how-to-insert-vertical-space-between-paragraphs
% https://latexref.xyz/_005cbigskip-_0026-_005cmedskip-_0026-_005csmallskip.html
\setlength{\parskip}{\smallskipamount}%
\setlength{\parindent}{0pt}%

% start new sections with new page
% https://tex.stackexchange.com/questions/9497/start-new-page-with-each-section
\AddToHook{cmd/section/before}{\clearpage}


% paragraphs and adding to ToC
% https://tex.stackexchange.com/questions/186981/is-there-a-subsubsubsection-command
\setcounter{tocdepth}{3}
\setcounter{secnumdepth}{4}

% https://tex.stackexchange.com/questions/5683/how-to-remove-top-and-bottom-whitespace-of-longtable
%\setlength{\LTpre}{0pt}
\setlength{\LTpost}{0pt}



% https://tex.stackexchange.com/questions/77809/how-do-i-remove-white-space-before-or-after-the-figures-and-tables
%\setlength{\floatsep}{18pt plus 18pt minus 18pt}
%\setlength{\textfloatsep}{18pt plus 18pt minus 18pt}
%\setlength{\intextsep}{18pt plus 18pt minus 18pt}



% wrapping within tabular etc.
% https://tex.stackexchange.com/questions/521976/how-to-wrap-text-in-tabular
\usepackage{makecell}       % thead
% https://tex.stackexchange.com/questions/417165/get-makecell-thead-to-emulate-font-size-of-table-environment
\renewcommand\theadfont{}   % wrong font size in thead


% https://tex.stackexchange.com/questions/168832/using-hypertarget-caption-in-hyperlink-caption
\let\oldhypertarget\hypertarget
\renewcommand{\hypertarget}[2]{%
  \oldhypertarget{#1}{#2}%
    \protected@write\@mainaux{}{%
        \string\expandafter\string\gdef
          \string\csname\string\detokenize{#1}\string\endcsname{#2}%
    }%
  }

\newcommand{\hyperlinkXY}[1]{%
  \hyperlink{#1}{\csname #1\endcsname}%
  }


% own stuff {{{


\newcommand{\img}[1]{\text{``#1''}}

\newcommand{\tableposleft}{0cm}
\newcommand{\tablevspaceASenum}{\vspace{1.5pt}}
\newcommand{\tablevspaceAStable}{\vspace{8pt}}

% https://tex.stackexchange.com/questions/50227/how-to-insert-spacing-for-start-of-paragraph
%\setlength{\parindent}{1em}
% https://tex.stackexchange.com/questions/32160/new-line-after-paragraph
%\newcommand{\aparagraph}[1]{\vspace{-1em}\paragraph{#1}\mbox{}\\\vspace{-1em}}
\newcommand{\aparagraph}[1]{\vspace{0em}\paragraph{#1}\mbox{}\\\vspace{-1em}}

\newcommand{\bitI}{\texttt{1}}
\newcommand{\bitO}{\texttt{0}}

%\newcommand{\tabref}[1]{\hyperlink{#1}{Tabelle \ref{#1}}}
%\newcommand{\tabref}[1]{\hyperlink{#1}{Tabelle}}
\newcommand{\tabref}[1]{Tabelle \ref{#1}}
\newcommand{\secref}[1]{Abschnitt \ref{#1}}
\newcommand{\imgref}[1]{Abbildung \ref{#1}}
\newcommand{\appref}[1]{Anhang \hyperlinkXY{#1}}

\usepackage{subfiles} % Best loaded last in the preamble

%\bibliographystyle{apacite}
% }}}
