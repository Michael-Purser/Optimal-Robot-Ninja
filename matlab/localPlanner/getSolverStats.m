function stats = getSolverStats(f)
    import casadi.*
    for k=0:f.n_instructions()-1
      op = f.instruction_id(k);
      if op==OP_CALL
        subf = f.instruction_MX(k).which_function();
        if subf.name()=='solver'
           stats = subf.stats(1);
        end
      end
    end
end