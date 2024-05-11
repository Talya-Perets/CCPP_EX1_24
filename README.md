תיעוד קוד גרף C++
מגישה: טליה פרץ תז: 322353780 מייל: talyape123@gmai.com
תיאור הפרויקט:
הפרויקט הזה מיישם מספר אלגוריתמים נפוצים על גרפים, המיוצגים באמצעות מטריצת שכנויות. הוא מורכב משתי מחלקות עיקריות:
מחלקת Graph:


פונקציה: loadGraph(const std::vector<std::vector<int>>& graph_matrix)
טעינת גרף ממטריצת שכנויות.
מקבלת מטריצת שכנויות כפרמטר ובודקת את תקינותה (גודל, ערכים).
מעדכנת את מטריצת השכנויות של מחלקת הגרף.
פונקציה: printGraph() const
הדפסת ייצוג הגרף.
מדפיסה את מטריצת השכנויות של הגרף את מספר הקודקודים ואת מספר הצלעות.
פונקציה: countEdges() const
עוברת על מטריצת השכנויות וסופרת את מספר הצלעות (תוך התחשבות אם הגרף מכוון).
פונקציה: isDirected() const
בדיקת כיווניות הגרף.
משווה את הערכים במטריצת השכנויות (האם לכל צלע יש צלע הפוכה).
מחזירה true אם הגרף מכוון, false אם לא מכוון.
פונקציה: isWeighted() const
בדיקת משקלות הצלעות בגרף.
עוברת על מטריצת השכנויות ובודקת אם יש ערכים שאינם 0 או 1.
מחזירה 1 אם הגרף ממושקל, 0 אם לא ממושקל, -1 אם יש משקל שלילי.
פונקציה: getGraphMatrix() const
החזרת מטריצת השכנויות של הגרף.
מחזירה עותק של מטריצת השכנויות של מחלקת הגרף.
מחלקת Algorithms:


פונקציה: isConnected(const Graph &g)
בודקת האם הגרף הוא קשיר או לא.
משתמשת בתוכנית חיפוש עומק (DFS) כדי לבדוק את קשירות הגרף.
אם הגרף הוא לא מכוון, היא משתמשת ב-DFS פעם אחת, ואם הוא מכוון, היא משתמשת ב-DFS פעמיים - פעם אחת מאחד קודקוד ראשי, ופעם נוספת מקודקוד שונה.
פונקציה: shortestPath(const Graph &g, int start, int end)
מוצאת את המסלול הקצר ביותר בין שני קודקודים בגרף.
משתמשת באלגוריתמים שונים בהתאם לתכונות הגרף:
BFS עבור גרפים לא ממושקלים.
Bellman-Ford עבור גרפים עם צלעות שליליות.
Dijkstra עבור גרפים עם צלעות חיוביות.
פונקציה: isContainsCycle(const Graph &g)
בודקת האם יש מעגלים בגרף.
משתמשת בתוכנית חיפוש עומק (DFS) כדי לבדוק האם יש מעגלים בגרף.
מפעילה DFS מכל קודקוד שלא נבדק עד כה.
פונקציה: isBipartite(const Graph &g)
בודקת האם הגרף הוא דו-חלוקתי או לא.
משתמשת בתוכנית חיפוש ברחב (BFS) כדי לצבוע את הגרף על פי שתי צבעים, כך שקודקודים שסמוכים אחד לשני יצבעו בצבעים שונים.
אם הגרף יכול להיות צבוע בכך, היא מחזירה את שני הקבוצות והצבעים שלהן.
פונקציה: negativeCycle(const Graph &g)
פונקציה זו בודקת האם יש מעגלים שליליים בגרף.
היא משתמשת באלגוריתם של Bellman-Ford כדי לבדוק האם יש מעגל שהמשקל שלו שלילי.
אם מצאה מעגל שלילי, היא מחזירה true, אחרת false.
פונקציות עזר(מוגדרות פרטיות ובשימוש רק בתוך המחלקה) : 
פונקציות עזר:
פונקציה: dfs(const std::vector<std::vector<int>> &graph_matrix, std::vector<bool> &visited, size_t current_node)
מבצעת סיור עומק (DFS) בגרף.
משתמשת בפריט נוכחי כדי לעבור על כל הקודקודים השכנים אליו.
במהלך הסיור, מעדכנת את הקודקודים שכבר ביקרה בהם במערך visited.
פונקציה: dfsForCycle(const std::vector<std::vector<int>> &graph_matrix, size_t v, std::vector<bool> &visited, std::vector<bool> &recStack, std::vector<size_t> &cycle, int parent)
מבצעת DFS בגרף כדי לאתר מעגלים.
משתמשת ברשומת קריאה לעקב אחר מעקב הביקור בקודקודים, כדי לזהות מעגלים.
כאשר מוצאת מעגל, מוסיפה את הקודקודים במעגל למערך cycle.
פונקציה: buildCycleString(const std::vector<size_t> &cycle)
בונה מחרוזת המייצגת את המעגל שנמצא בגרף.
מקבלת את המערך cycle שמכיל את הקודקודים במעגל.
מחזירה מחרוזת המייצגת את המעגל, כאשר הקודקודים מופרדים במחרוזת באמצעות סימן החצים (->).
פונקציה: BFS(const Graph& g, size_t start, size_t end)
מחפשת מסלול קצר ביותר בין שני צמתים באמצעות סיור BFS.
מקבלת את הגרף, את הצומת ההתחלתי ואת הצומת הסופי.
פונקציה: Dijkstra(const Graph& g, size_t start, size_t end)
מחשבת את מסלול הקצר ביותר בין שני צמתים באמצעות אלגוריתם דייקסטרה.
מקבלת את הגרף, את הצומת ההתחלתי ואת הצומת הסופי.
פונקציה: BellmanFord(const Graph& g, size_t start, size_t end)
מוצאת את מסלול הקצר ביותר בין שני צמתים, כולל צמתים שבהם יש משקלים שליליים, באמצעות אלגוריתם בלמן-פורד.
מקבלת את הגרף, את הצומת ההתחלתי ואת הצומת הסופי.
קבצי בדיקה : 
demo: מכיל דוגמאות של קבצים ופלטים של הפונקציות.
test: מכיל מס טסטים לכל הפונקציות ה-public.
הקומפיילר:
g++
הפעלה:
make run - להריץ את קובץ ה-demo
make runtest - להריץ את קובץ הטסט


 
ד
