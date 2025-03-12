function drawdomain(domain, lnt, lnw)

    xmin = domain(1,1);
    xmax = domain(1,2);
    ymin = domain(2,1);
    ymax = domain(2,2);

    plot([xmin xmax], [ymin ymin], num2str(lnt), 'linewidth', lnw); 
    hold on;
    plot([xmax xmax], [ymin ymax], num2str(lnt), 'linewidth', lnw);
    plot([xmax xmin], [ymax ymax], num2str(lnt), 'linewidth', lnw);
    plot([xmin xmin], [ymax ymin], num2str(lnt), 'linewidth', lnw);
end