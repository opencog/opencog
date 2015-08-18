This directory contains those files which were neccessary to extract information about the data. The tools to plot graphs are in the folder <i>graphing tools</i>. These files were used to plot graphs when I had around 1.2million word pairs. As the number of word pairs grow, they will probably need to be plotted again. A description of the files follows.

<h1>Frequency.scm</h1>
This file sorts all the wordpairs according to their frequency. Wordpairs are gathered into a list. They are arranged in ascending/descending order. The wordpairs are then removed from the list so that only the frequencies remain. These frequencies are then redirected to a file. A graph is plotted from the output.

<h1>top50-mi.scm</h1>
This file sorts wordpairs according to their mutual information. Then it takes the top 50 wordpairs. The code in this file can be extended to take the top <b>n</b> wordpairs when they are sorted according to the mutual information. Before applying this code, it is necessary to remove the wordpairs containing "anyword" as a wordnode. This is because, such wordpairs are used only for calculating mutual information. They aren't used for anything else. If such wordpairs are not removed, the output can be misleading.
Also, some wordpairs are observed very few times. This leads to a high mutual information. To prevent this, this file also contains functions to filter wordpairs according to their frequency of occurence.
Again, the output is redirected to a file and a graph is plotted.

<h1>words.scm</h1>
This file has the functions to find the nth word and then get the number of times that this word has occured in wordpairs. This is an immensely slow process, since the functions have to loop over millions of wordpairs. Practically speaking, this function cannot be run for every word. Therefore, the words are sampled according to a formula and their frequency is also noted. These values are again plotted to a graph.